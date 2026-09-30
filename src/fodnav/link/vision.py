"""The vision link: Bthcorn's ``fod-vision`` library, adapted to nav's needs.

**It is a library, not a service.** There is no MQTT topic, no socket and no
daemon: you import ``fodcv.runtime.vision.Vision``, it runs a capture thread
inside your process, and you read it whenever your loop gets round to it. The
three-process MQTT design this repo was originally written against never
existed -- see ``docs/vendor/fod-vision-v0.3.0-INTEGRATION.md``.

**What he provides and what he does not.** He gives tracked, hysteresis-smoothed
detections with stable ids and boxes **in pixels**. He is explicit that he will
not give floor coordinates:

    "No metres, no floor coordinates. Boxes are pixels. Projecting to the ground
     needs a mount height and tilt the package cannot know."
    "No steering and no pick point."

That is exactly the boundary CLAUDE.md §1 draws, so the division of labour is
agreed rather than negotiated: **he says what is on the screen, we say where it
is on the floor and what the wheels should do about it.**

His headline read is ``zone_blocked()`` -- a tripwire across a horizontal band of
the *image*, ignoring x entirely -- which suits a robot that cannot steer. Nav
chases, so nav consumes ``latest()`` and does its own projection. ``zone_blocked``
is still carried through for the log and for a speed-governed mode.

Two of his rules shape this module:

* **Do not branch on ``cls``.** It flips between frames on a live track, and the
  four class names disappear when the single-class arena dataset lands. Branch on
  ``state`` and ``action``, which survive that change.
* **A dead camera raises; it does not return an empty list.** "No debris, keep
  patrolling" must never be what a broken camera looks like. Failures are caught
  here and turned into a frame carrying ``error``, so the control loop sees a
  reason rather than silence -- and the vision timeout still fires underneath.
"""

from __future__ import annotations

import json
import math
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterator, Protocol, Sequence

__all__ = [
    "CONFIRM", "CAUTION", "IGNORE", "PICK", "REPORT",
    "Target", "VisionFrame", "VisionError",
    "VisionSource", "LibraryVisionSource", "QueueVisionSource", "ReplayVisionSource",
    "JsonlVisionLog", "parse_detail", "select_targets", "iter_jsonl",
]

#: ``state`` values. Hysteresis is his: CONFIRM at raw >= 0.5, latched until the
#: smoothed score falls below 0.25. Nav does not re-implement this.
CONFIRM, CAUTION, IGNORE = "CONFIRM", "CAUTION", "IGNORE"
#: ``action`` values. Every shipped class maps to PICK today; REPORT will not
#: fire until a detector exists that can tell ferrous from non-ferrous (his
#: FR-13 vs FR-3 note). Do not write code that depends on seeing REPORT.
PICK, REPORT = "PICK", "REPORT"


class VisionError(RuntimeError):
    """The capture thread died. Terminal -- a ``Vision`` cannot be restarted."""


@dataclass(frozen=True, slots=True)
class Target:
    """One tracked object, mirroring his ``Target`` namedtuple exactly."""

    id: int
    state: str
    action: str
    cls: str
    conf: float
    box: tuple[int, int, int, int]        # x0, y0, x1, y1 -- top-left origin, y down
    centroid: tuple[float, float]

    @property
    def ground_px(self) -> tuple[float, float]:
        """Bottom-centre of the box: where the object meets the floor.

        **Not ``centroid``.** His ``centroid`` is the box *centre*, which is the
        exact mistake CLAUDE.md §3 warns about -- projecting it puts the object
        further away by an amount that grows with its height and with range, and
        the error looks like a bad calibration for days. The bottom edge is the
        contact point, and his ``box`` gives it to us as ``y1``.
        """
        x0, _y0, x1, y1 = self.box
        return (0.5 * (x0 + x1), float(y1))

    @property
    def confirmed(self) -> bool:
        return self.state == CONFIRM

    @property
    def pickable(self) -> bool:
        return self.action == PICK


@dataclass(frozen=True, slots=True)
class VisionFrame:
    """One read of the library, in the shape the control loop wants."""

    frame_id: int
    age_s: float
    blocked: bool
    frame_size: tuple[int, int]
    targets: tuple[Target, ...]
    error: str | None = None
    t_recv_monotonic: float = 0.0

    @property
    def ok(self) -> bool:
        return self.error is None

    @property
    def width_px(self) -> int:
        return self.frame_size[0]

    @property
    def height_px(self) -> int:
        return self.frame_size[1]


def select_targets(
    frame: VisionFrame,
    states: Sequence[str] = (CONFIRM,),
    actions: Sequence[str] = (PICK,),
) -> tuple[Target, ...]:
    """Filter on ``state`` and ``action`` -- never on ``cls``.

    His class names (``bolt``/``nut``/``screw``/``washer``) are diagnostic: they
    flip frame to frame on one object, and they stop existing when the
    single-class arena dataset lands. ``state`` and ``action`` survive that.

    This replaces the old ``target_classes``/``ignore_classes`` filtering, which
    was written against a schema that never shipped and named a ``nail`` class
    that does not exist.
    """
    want_state, want_action = set(states), set(actions)
    return tuple(t for t in frame.targets if t.state in want_state and t.action in want_action)


# ---------------------------------------------------------------------------
# his detail() dict is our log and replay format
# ---------------------------------------------------------------------------


def parse_detail(d: dict[str, Any], t_recv: float | None = None) -> VisionFrame:
    """Build a :class:`VisionFrame` from his ``detail()`` dictionary.

    ``detail()`` is JSON-serialisable, never raises, and carries every field
    from one frame -- so it is used verbatim as the on-disk format. Nav invents
    no schema of its own here, which is the whole reason the two sides cannot
    drift apart: there is only one shape and he owns it.

    ``detail()["tracks"]`` includes tracks *coasting* through missed frames;
    ``latest()`` is the subset with ``misses == 0``. This reconstructs
    ``latest()`` so a replayed frame matches a live one.
    """
    cam = d.get("camera") or {}
    size = cam.get("frame_size") or [0, 0]
    targets: list[Target] = []
    for tr in d.get("tracks") or []:
        if tr.get("misses", 0) != 0:
            continue                       # coasting: not in latest()
        box = tuple(int(v) for v in tr["box"])
        cx, cy = tr.get("centroid") or (0.5 * (box[0] + box[2]), 0.5 * (box[1] + box[3]))
        targets.append(
            Target(
                id=int(tr["id"]), state=str(tr["state"]), action=str(tr["action"]),
                cls=str(tr.get("cls", "")), conf=float(tr.get("conf", 0.0)),
                box=box, centroid=(float(cx), float(cy)),
            )
        )
    age = d.get("age", math.inf)
    return VisionFrame(
        frame_id=int(d.get("frame_id", -1)),
        age_s=math.inf if age is None else float(age),
        blocked=bool(d.get("blocked", False)),
        frame_size=(int(size[0]), int(size[1])),
        targets=tuple(targets),
        error=d.get("error"),
        t_recv_monotonic=time.monotonic() if t_recv is None else t_recv,
    )


class JsonlVisionLog:
    """Append every ``detail()`` verbatim, one JSON object per line.

    ``fodnav-replay`` feeds these back through the same interface, so the whole
    nav stack is testable against real detector output with no camera and no
    robot -- which is the cheapest way to find out whether our projection agrees
    with his boxes.
    """

    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._fh = self.path.open("a", encoding="utf-8")
        self.n_written = 0

    def write(self, detail: dict[str, Any]) -> None:
        try:
            self._fh.write(json.dumps(detail, separators=(",", ":"), default=str) + "\n")
            self._fh.flush()   # a log lost to a SIGKILL is not a log
            self.n_written += 1
        except Exception:
            # Deliberately broad. A full disk, a read-only card, a closed
            # handle -- none of them are worth stopping a robot for, and the
            # narrow tuple this used to catch missed ValueError from a closed
            # file. The log is evidence, not a dependency.
            pass

    def close(self) -> None:
        try:
            self._fh.close()
        except Exception:
            pass

    def __enter__(self) -> "JsonlVisionLog":
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()


def iter_jsonl(path: str | Path) -> Iterator[dict[str, Any]]:
    with Path(path).open("r", encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                rec = json.loads(line)
            except json.JSONDecodeError:
                continue
            if isinstance(rec, dict):
                yield rec


# ---------------------------------------------------------------------------
# sources
# ---------------------------------------------------------------------------


@dataclass
class SourceStats:
    frames: int = 0
    targets_seen: int = 0
    errors: int = 0
    last_error: str = ""

    def as_dict(self) -> dict[str, Any]:
        return dict(self.__dict__)


class VisionSource(Protocol):
    """Where detections come from. The sim, a replayed log, or the real library."""

    def start(self) -> None: ...
    def stop(self) -> None: ...
    def poll(self) -> list[VisionFrame]: ...


class _BaseSource:
    def __init__(self, maxlen: int | None = 64) -> None:
        self.stats = SourceStats()
        # Bounded for live sources: if the control loop stalls, the oldest
        # frames are the ones worth losing, because acting on a stale box is
        # worse than not acting. Replay passes None -- dropping frames there
        # would silently discard most of a recorded log.
        self._q: deque[VisionFrame] = deque(maxlen=maxlen)
        self.n_dropped = 0

    def _offer(self, frame: VisionFrame) -> None:
        if self._q.maxlen is not None and len(self._q) == self._q.maxlen:
            self.n_dropped += 1
        self.stats.frames += 1
        self.stats.targets_seen += len(frame.targets)
        if frame.error:
            self.stats.errors += 1
            self.stats.last_error = frame.error
        self._q.append(frame)

    def poll(self) -> list[VisionFrame]:
        out = list(self._q)
        self._q.clear()
        return out


class QueueVisionSource(_BaseSource):
    """Fed by hand. For tests and for the simulator's synthetic camera."""

    def start(self) -> None:  # pragma: no cover - nothing to do
        pass

    def stop(self) -> None:  # pragma: no cover - nothing to do
        pass

    def offer(self, frame: VisionFrame | dict) -> None:
        self._offer(frame if isinstance(frame, VisionFrame) else parse_detail(frame))


class LibraryVisionSource(_BaseSource):
    """The real thing: his ``Vision`` object, polled from our control loop.

    ``fodcv`` is imported **inside** :meth:`start`, not at module scope, for the
    same reason ``pyserial`` is: it needs ``picamera2`` and
    ``hailo_platform``, which exist only on the Pi. Keeping the import late is
    what lets ``import fodnav`` work on a laptop and the whole test suite run
    with no hardware (CLAUDE.md §2).

    One process owns the accelerator, and a ``Vision`` is single-use -- so this
    class constructs exactly one and never retries a failed one.
    """

    def __init__(
        self,
        hef: str,
        lookahead: tuple[float, float] = (0.5, 1.0),
        log: JsonlVisionLog | None = None,
        **vision_kwargs: Any,
    ) -> None:
        super().__init__()
        self.hef = hef
        self.lookahead = lookahead
        self.log = log
        self.kwargs = vision_kwargs
        self._vision = None
        self._cm = None
        self.failed = False
        self._last_frame_id: int | None = None
        self.n_repeats = 0      # polls that found no new frame: normal at 50 Hz vs 30 FPS

    def start(self) -> None:
        from fodcv.runtime.vision import Vision  # noqa: PLC0415 - Pi-only, see docstring

        self._cm = Vision(hef=self.hef, lookahead=self.lookahead, **self.kwargs)
        # His lifecycle is a context manager: __enter__ opens the camera and
        # waits up to 30 s for the Hailo. Skipping it leaves the device claimed
        # by a dead process, so it is entered and exited explicitly.
        self._vision = self._cm.__enter__()

    def attach(self, vision: Any) -> None:
        """Read from an object that already behaves like an entered ``Vision``.

        For the simulator and tests: :meth:`start` is the real path, and it is
        the only one that imports ``fodcv``. Anything with ``detail()`` and
        ``age`` will do, which is what lets the laptop exercise *this* class --
        the one that runs on the Pi -- rather than a stand-in for it.
        """
        self._vision = vision

    def stop(self) -> None:
        if self._cm is not None:
            try:
                self._cm.__exit__(None, None, None)
            except Exception:
                pass
            self._cm = self._vision = None

    def poll(self) -> list[VisionFrame]:
        """Read one frame, and offer it only if it is new. Never raises into the loop.

        His reads re-raise the capture thread's exception, deliberately, so that
        a dead camera cannot look like a clear floor. That exception is caught
        here and carried on the frame as ``error`` -- the FSM stops on it.

        ``detail()`` returns the last *completed* frame on every call. So a
        camera that has stalled -- thread alive, no new frames -- answers every
        poll with the same frame and a growing ``age``, which looks exactly like
        a healthy camera read twice. Offering it again would keep the vision
        heartbeat alive on a frozen picture, and nav would chase whatever was
        in it. A frame is therefore offered once, when its ``frame_id`` is new;
        a stall then means no frames, the heartbeat lapses after
        ``loop.vision_timeout_ms``, and the FSM stops. Before his first frame
        ``age`` is infinite and nothing is offered, for the same reason.
        """
        if self._vision is None or self.failed:
            return []
        try:
            detail = self._vision.detail()          # never raises; has error inside
            frame = parse_detail(detail)
            if frame.error:
                self.failed = True                  # terminal: build a new source
            elif math.isinf(frame.age_s) or frame.frame_id == self._last_frame_id:
                self.n_repeats += 1
                return super().poll()
            self._last_frame_id = frame.frame_id
            if self.log is not None:
                self.log.write(detail)              # one line per frame, not per poll
            self._offer(frame)
        except Exception as e:                       # pragma: no cover - hardware only
            self.failed = True
            self._offer(
                VisionFrame(-1, math.inf, False, (0, 0), (), error=f"{type(e).__name__}: {e}")
            )
        return super().poll()

    @property
    def age_s(self) -> float:
        """Seconds since his last completed frame. Never raises; only grows."""
        if self._vision is None:
            return math.inf
        try:
            return float(self._vision.age)
        except Exception:  # pragma: no cover - hardware only
            return math.inf


class ReplayVisionSource(_BaseSource):
    """Feed a recorded ``detail()`` log back through the same interface."""

    def __init__(self, path: str | Path, speed: float = 0.0, loop: bool = False) -> None:
        super().__init__(maxlen=None)   # a replayed log is read whole, never dropped
        self.path = Path(path)
        self.speed = float(speed)
        self.loop = loop
        self._records: list[dict[str, Any]] = []
        self._i = 0
        self._t0 = 0.0

    def start(self) -> None:
        self._records = list(iter_jsonl(self.path))
        self._i = 0
        self._t0 = time.monotonic()

    def stop(self) -> None:
        self._records = []

    @property
    def finished(self) -> bool:
        return self._i >= len(self._records) and not self.loop

    def poll(self) -> list[VisionFrame]:
        if not self._records:
            return []
        if self.speed <= 0.0:
            due = self._records[self._i :]
            self._i = len(self._records)
        else:
            # His frames arrive at a fixed 30 FPS, so elapsed frames is elapsed
            # time times the rate -- there is no receipt timestamp to honour.
            want = int((time.monotonic() - self._t0) * self.speed * 30.0)
            due = self._records[self._i : max(self._i, min(want, len(self._records)))]
            self._i += len(due)
            if self.loop and self._i >= len(self._records):
                self._i = 0
                self._t0 = time.monotonic()
        for rec in due:
            self._offer(parse_detail(rec))
        return super().poll()
