"""The vision link: MQTT subscriber, message parsing, JSONL log, replay.

**Every field name in the detection schema appears in this module and nowhere
else.** That is the whole point of the module. The schema in CLAUDE.md section 8
is being *requested* of Bthcorn, not implemented by him -- as of now
``camera_hailo.py`` renders a preview and publishes nothing, and the only
publisher in existence is ``tools/fake_detections.py``. When the real one
lands, whatever it differs in should be a change here and nowhere else.

Schema version 1, on topic ``fod/detections``, QoS 0, retain false::

    {"schema": 1,
     "t_capture": 1756000000.123, "t_publish": 1756000000.156,
     "frame_id": 4821, "frame_size": [2304, 1296],
     "dets": [{"cls": "bolt", "conf": 0.87, "bbox": [1102, 812, 61, 44]}]}

``bbox`` is ``[x, y, w, h]`` in **original captured frame** pixels, top-left
origin, never the letterboxed 480x480 network input. ``t_capture`` is the
sensor timestamp, not the publish time: nav ages the estimate against odometry
with it.

Two clocks, and they are not interchangeable:

* ``t_capture`` / ``t_publish`` are the *publisher's* wall clock. Same host in
  production, so they are comparable to ``time.time()`` here, and that is what
  ages a detection against odometry. Under replay they are historical and mean
  nothing in the present.
* Receipt times are ``time.monotonic()``. The vision heartbeat timeout uses
  those and only those, because an NTP step must not be able to convince the
  robot that vision died -- or that it is alive.

Nothing here blocks the control loop. The MQTT client runs paho's own network
thread, which appends raw payloads to a bounded deque; ``poll()`` drains it.
File I/O for the JSONL log happens on that thread too, never on the caller's.
"""

from __future__ import annotations

import json
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable, Iterator, Protocol, Sequence

__all__ = [
    "SCHEMA_VERSION",
    "DEFAULT_TOPIC",
    "Detection",
    "DetectionFrame",
    "DetectionSchemaError",
    "DetectionSource",
    "MqttDetectionSource",
    "ReplayDetectionSource",
    "QueueDetectionSource",
    "JsonlDetectionLog",
    "parse_message",
    "select_targets",
    "iter_jsonl",
]

#: Schema version this module speaks. A publisher sending anything else is
#: rejected outright rather than parsed hopefully.
SCHEMA_VERSION = 1

#: Default MQTT topic. The configured value in ``config/nav.yaml`` wins; this
#: is here so the fake publisher and the subscriber cannot drift apart.
DEFAULT_TOPIC = "fod/detections"


class DetectionSchemaError(ValueError):
    """A message that does not conform. Counted and logged, never fatal.

    A malformed message is a vision-side bug or a version skew. Either way the
    right response is to drop the message and keep the control loop running --
    the vision timeout will stop the robot if they *all* stop conforming.
    """


@dataclass(frozen=True, slots=True)
class Detection:
    """One box, in original-frame pixels."""

    cls: str
    conf: float
    x: float
    y: float
    w: float
    h: float

    @property
    def bbox(self) -> tuple[float, float, float, float]:
        return (self.x, self.y, self.w, self.h)

    @property
    def ground_px(self) -> tuple[float, float]:
        """Bottom-centre of the box: where the object meets the floor.

        Not the centroid. The floor is the plane being intersected and the
        bottom edge is where the object touches it; using the centroid adds a
        range error proportional to object height, and it will look exactly
        like a calibration problem (CLAUDE.md section 3).
        """
        return (self.x + 0.5 * self.w, self.y + self.h)

    @property
    def centre_px(self) -> tuple[float, float]:
        """Only for drawing. Never for projection."""
        return (self.x + 0.5 * self.w, self.y + 0.5 * self.h)


@dataclass(frozen=True, slots=True)
class DetectionFrame:
    """One parsed message.

    An empty ``dets`` is a perfectly good message and means "the floor is
    clear". It is not the same thing as no message, which means vision is dead.
    """

    t_capture: float
    t_publish: float
    frame_id: int
    frame_size: tuple[int, int]
    dets: tuple[Detection, ...]
    t_recv_monotonic: float = 0.0

    @property
    def width_px(self) -> int:
        return self.frame_size[0]

    @property
    def height_px(self) -> int:
        return self.frame_size[1]

    def age_s(self, now: float | None = None) -> float:
        """Seconds since capture, on the publisher's wall clock.

        Use for aging an estimate against odometry, never for the heartbeat.
        """
        return (time.time() if now is None else now) - self.t_capture

    def pipeline_latency_s(self) -> float:
        """Capture to publish, as the publisher measured it."""
        return self.t_publish - self.t_capture


def _num(d: Any, key: str, where: str) -> float:
    try:
        v = d[key]
    except (KeyError, TypeError):
        raise DetectionSchemaError(f"{where}: missing {key!r}") from None
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        raise DetectionSchemaError(f"{where}: {key} must be a number, got {v!r}")
    return float(v)


def parse_message(payload: bytes | str | dict) -> DetectionFrame:
    """Parse one MQTT payload into a :class:`DetectionFrame`.

    Strict about the things a wrong answer would be silent about -- the schema
    version, the frame size, the bbox arity -- and indifferent to extra fields,
    so that a publisher which grows a field does not take the robot down.
    """
    if isinstance(payload, (bytes, bytearray)):
        try:
            payload = payload.decode("utf-8")
        except UnicodeDecodeError as e:
            raise DetectionSchemaError(f"payload is not utf-8: {e}") from None
    if isinstance(payload, str):
        try:
            msg = json.loads(payload)
        except json.JSONDecodeError as e:
            raise DetectionSchemaError(f"payload is not JSON: {e}") from None
    else:
        msg = payload
    if not isinstance(msg, dict):
        raise DetectionSchemaError(f"payload must be a JSON object, got {type(msg).__name__}")

    schema = msg.get("schema")
    if schema != SCHEMA_VERSION:
        raise DetectionSchemaError(
            f"schema version {schema!r}, expected {SCHEMA_VERSION}. The publisher and "
            f"src/fodnav/link/detections.py disagree; one of them was changed without "
            f"the other."
        )

    t_capture = _num(msg, "t_capture", "message")
    t_publish = float(msg.get("t_publish", t_capture))

    frame_id = msg.get("frame_id", -1)
    if isinstance(frame_id, bool) or not isinstance(frame_id, int):
        raise DetectionSchemaError(f"frame_id must be an integer, got {frame_id!r}")

    size = msg.get("frame_size")
    if (
        not isinstance(size, (list, tuple))
        or len(size) != 2
        or any(isinstance(s, bool) or not isinstance(s, int) or s <= 0 for s in size)
    ):
        raise DetectionSchemaError(
            f"frame_size must be [width, height] in positive integer pixels, got {size!r}. "
            f"Without it there is no way to check that the projection is being asked "
            f"about the resolution it was calibrated at."
        )

    raw_dets = msg.get("dets", [])
    if not isinstance(raw_dets, list):
        raise DetectionSchemaError(f"dets must be a list, got {type(raw_dets).__name__}")

    dets: list[Detection] = []
    for i, d in enumerate(raw_dets):
        where = f"dets[{i}]"
        if not isinstance(d, dict):
            raise DetectionSchemaError(f"{where} must be an object, got {type(d).__name__}")
        cls = d.get("cls")
        if not isinstance(cls, str):
            raise DetectionSchemaError(f"{where}: cls must be a string, got {cls!r}")
        conf = _num(d, "conf", where)
        bbox = d.get("bbox")
        if not isinstance(bbox, (list, tuple)) or len(bbox) != 4:
            raise DetectionSchemaError(f"{where}: bbox must be [x, y, w, h], got {bbox!r}")
        x, y, w, h = (float(v) for v in bbox)
        if w <= 0 or h <= 0:
            raise DetectionSchemaError(f"{where}: bbox has non-positive size {bbox!r}")
        dets.append(Detection(cls=cls, conf=conf, x=x, y=y, w=w, h=h))

    return DetectionFrame(
        t_capture=t_capture,
        t_publish=t_publish,
        frame_id=frame_id,
        frame_size=(int(size[0]), int(size[1])),
        dets=tuple(dets),
        t_recv_monotonic=time.monotonic(),
    )


def select_targets(
    frame: DetectionFrame,
    target_classes: Iterable[str],
    ignore_classes: Iterable[str] = (),
    min_conf: float = 0.0,
) -> tuple[Detection, ...]:
    """Apply the nav-side class rules from CLAUDE.md section 8.

    ``nail``, ``screw`` and ``bolt`` are **one** target class. The CV repo's own
    results record that screws are found reliably and labelled ``bolt``
    reliably, and PRD FR-3 asks for a single ``metal_fastener`` anyway.
    Nothing downstream may branch on which of the three came back.

    ``unknown`` is dropped. It is 53% of the training data, a grab-bag of four
    shapes, and the class that fires on furniture; the CV repo suppresses it by
    default and nav must not resurrect it.
    """
    targets = {c.lower() for c in target_classes}
    ignored = {c.lower() for c in ignore_classes}
    return tuple(
        d
        for d in frame.dets
        if d.cls.lower() in targets and d.cls.lower() not in ignored and d.conf >= min_conf
    )


# ---------------------------------------------------------------------------
# JSONL log
# ---------------------------------------------------------------------------


class JsonlDetectionLog:
    """Append every message received, verbatim, one JSON object per line.

    The envelope records what arrived and when it arrived here; the payload is
    stored unmodified, including payloads that failed to parse, because those
    are the ones worth having later. ``fodnav-replay`` feeds this back through
    the same interface, so the whole nav stack is testable against real vision
    output with no camera, no Pi and no robot.
    """

    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._fh = self.path.open("a", encoding="utf-8")
        self.n_written = 0

    def write(self, payload: bytes | str, topic: str = DEFAULT_TOPIC, error: str | None = None) -> None:
        if isinstance(payload, (bytes, bytearray)):
            text = payload.decode("utf-8", errors="replace")
        else:
            text = payload
        record: dict[str, Any] = {
            "t_recv": time.time(),
            "t_recv_monotonic": time.monotonic(),
            "topic": topic,
        }
        try:
            record["msg"] = json.loads(text)
        except json.JSONDecodeError:
            record["raw"] = text
        if error:
            record["error"] = error
        self._fh.write(json.dumps(record, separators=(",", ":")) + "\n")
        self._fh.flush()  # a log that is lost when the process is SIGKILLed is not a log
        self.n_written += 1

    def close(self) -> None:
        try:
            self._fh.close()
        except Exception:
            pass

    def __enter__(self) -> "JsonlDetectionLog":
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()


def iter_jsonl(path: str | Path) -> Iterator[dict[str, Any]]:
    """Yield envelope records from a detection JSONL log, skipping junk lines."""
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


class DetectionSource(Protocol):
    """Where detections come from.

    MQTT was chosen because Mosquitto is already in the stack and a loopback
    publish is sub-millisecond. If jitter ever matters, a UNIX domain socket is
    an acceptable swap -- and the swap is local to this file precisely because
    everything upstream depends on this interface and not on paho.
    """

    def start(self) -> None: ...
    def stop(self) -> None: ...
    def poll(self) -> list[DetectionFrame]: ...
    @property
    def last_recv_monotonic(self) -> float | None: ...


@dataclass
class SourceStats:
    received: int = 0
    parsed: int = 0
    malformed: int = 0
    dropped: int = 0  # queue overflow: nav was not draining fast enough
    last_error: str = ""

    def as_dict(self) -> dict[str, Any]:
        return {
            "received": self.received,
            "parsed": self.parsed,
            "malformed": self.malformed,
            "dropped": self.dropped,
            "last_error": self.last_error,
        }


class _BaseSource:
    def __init__(self, maxlen: int = 256) -> None:
        self._q: deque[tuple[float, bytes | str]] = deque(maxlen=maxlen)
        self.stats = SourceStats()
        self._last_recv: float | None = None

    @property
    def last_recv_monotonic(self) -> float | None:
        """Monotonic receipt time of the last message, parsed or not.

        A malformed message still proves vision is alive, so the heartbeat
        counts it. Whether it is *useful* is a separate question the parse
        counters answer.
        """
        return self._last_recv

    def seconds_since_last(self) -> float:
        """Age of the vision heartbeat. ``inf`` if nothing has ever arrived."""
        if self._last_recv is None:
            return float("inf")
        return time.monotonic() - self._last_recv

    def _offer(self, payload: bytes | str) -> None:
        """Called from a non-control-loop thread. Must not block on anything."""
        if len(self._q) == self._q.maxlen:
            self.stats.dropped += 1
        self._q.append((time.monotonic(), payload))
        self._last_recv = time.monotonic()
        self.stats.received += 1

    def _drain(self) -> list[DetectionFrame]:
        out: list[DetectionFrame] = []
        while self._q:
            _, payload = self._q.popleft()
            try:
                out.append(parse_message(payload))
                self.stats.parsed += 1
            except DetectionSchemaError as e:
                self.stats.malformed += 1
                self.stats.last_error = str(e)
        return out


class QueueDetectionSource(_BaseSource):
    """A source fed by hand. For tests, and for the simulator's synthetic camera."""

    def start(self) -> None:  # pragma: no cover - nothing to do
        pass

    def stop(self) -> None:  # pragma: no cover - nothing to do
        pass

    def offer(self, payload: bytes | str | dict) -> None:
        if isinstance(payload, dict):
            payload = json.dumps(payload)
        self._offer(payload)

    def poll(self) -> list[DetectionFrame]:
        return self._drain()


class MqttDetectionSource(_BaseSource):
    """Subscribe to the vision process over MQTT.

    paho's network thread does the receiving and (optionally) the JSONL
    writing; ``poll()`` on the control-loop thread only pops from a deque. The
    queue is bounded: if nav stalls, the *oldest* detections are lost and the
    drop is counted, because acting on a stale box is worse than not acting.
    """

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 1883,
        topic: str = DEFAULT_TOPIC,
        log: JsonlDetectionLog | None = None,
        maxlen: int = 256,
        client_id: str = "fodnav",
    ) -> None:
        super().__init__(maxlen=maxlen)
        self.host, self.port, self.topic = host, port, topic
        self.log = log
        self.connected = False
        self._client_id = client_id
        self._client = None

    def start(self) -> None:
        import paho.mqtt.client as mqtt  # imported late: the sim never needs a broker

        try:
            client = mqtt.Client(
                mqtt.CallbackAPIVersion.VERSION2, client_id=self._client_id, clean_session=True
            )
        except AttributeError:  # pragma: no cover - paho 1.x
            client = mqtt.Client(client_id=self._client_id, clean_session=True)

        def on_connect(client, userdata, flags, reason_code, properties=None):
            self.connected = True
            client.subscribe(self.topic, qos=0)

        def on_disconnect(client, userdata, *args, **kwargs):
            self.connected = False

        def on_message(client, userdata, msg):
            if self.log is not None:
                try:
                    self.log.write(msg.payload, topic=msg.topic)
                except Exception as e:  # a broken log must not kill the link
                    self.stats.last_error = f"log write failed: {e}"
            self._offer(msg.payload)

        client.on_connect = on_connect
        client.on_disconnect = on_disconnect
        client.on_message = on_message
        # connect_async + loop_start: never block the caller on a broker that
        # is not up yet, and reconnect on its own if it goes away.
        client.connect_async(self.host, self.port, keepalive=10)
        client.loop_start()
        self._client = client

    def stop(self) -> None:
        if self._client is not None:
            try:
                self._client.loop_stop()
                self._client.disconnect()
            except Exception:
                pass
            self._client = None

    def poll(self) -> list[DetectionFrame]:
        return self._drain()


class ReplayDetectionSource(_BaseSource):
    """Feed a recorded JSONL log back through the same interface.

    Real time by default: messages come out spaced as they arrived, so timing
    bugs reproduce. ``speed`` scales that, and ``speed=0`` releases everything
    on the first poll for a fast unit test.

    The recorded ``t_capture`` values are historical. ``rebase_capture_time``
    shifts them onto the present clock so that consumers which age a detection
    against odometry see plausible ages instead of a message from last Tuesday.
    """

    def __init__(
        self,
        path: str | Path,
        speed: float = 1.0,
        loop: bool = False,
        rebase_capture_time: bool = True,
    ) -> None:
        super().__init__(maxlen=1 << 20)
        self.path = Path(path)
        self.speed = float(speed)
        self.loop = loop
        self.rebase = rebase_capture_time
        self._records: list[tuple[float, str]] = []
        self._i = 0
        self._t0_wall = 0.0
        self._t0_mono = 0.0
        # Resolved from the first frame actually parsed, not from the receipt
        # envelope: the offset that matters is the one on t_capture, which is
        # the field consumers age against odometry.
        self._capture_offset: float | None = None

    def start(self) -> None:
        recs: list[tuple[float, str]] = []
        for rec in iter_jsonl(self.path):
            if "msg" in rec:
                payload = json.dumps(rec["msg"], separators=(",", ":"))
            elif "raw" in rec:
                payload = str(rec["raw"])
            else:
                continue
            recs.append((float(rec.get("t_recv", 0.0)), payload))
        self._records = recs
        self._i = 0
        self._t0_wall = recs[0][0] if recs else 0.0
        self._t0_mono = time.monotonic()
        self._capture_offset = None if self.rebase else 0.0

    def stop(self) -> None:
        self._records = []

    @property
    def finished(self) -> bool:
        return self._i >= len(self._records) and not self.loop

    def poll(self) -> list[DetectionFrame]:
        if not self._records:
            return []
        if self.speed <= 0.0:
            due = self._records[self._i :]
            self._i = len(self._records)
        else:
            elapsed = (time.monotonic() - self._t0_mono) * self.speed
            due = []
            while self._i < len(self._records):
                t_rel = self._records[self._i][0] - self._t0_wall
                if t_rel > elapsed:
                    break
                due.append(self._records[self._i])
                self._i += 1
            if self.loop and self._i >= len(self._records):
                self._i = 0
                self._t0_mono = time.monotonic()
        for _, payload in due:
            self._offer(payload)
        frames = self._drain()
        if self._capture_offset is None and frames:
            self._capture_offset = time.time() - frames[0].t_capture
        if self._capture_offset:
            frames = [
                DetectionFrame(
                    t_capture=f.t_capture + self._capture_offset,
                    t_publish=f.t_publish + self._capture_offset,
                    frame_id=f.frame_id,
                    frame_size=f.frame_size,
                    dets=f.dets,
                    t_recv_monotonic=f.t_recv_monotonic,
                )
                for f in frames
            ]
        return frames
