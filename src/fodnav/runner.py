"""The control loop. One place, used by both the simulator and the real robot.

The loop's first obligation is not navigation, it is the heartbeat.
``docs/protocol.md`` section 6: the ESP32 zeroes velocity if it has not
received a ``V`` for 300 ms, and nav's side of that contract is a ``V`` at a
fixed 50 Hz whenever the link is open -- including ``V 0.000 0.000``, including
when nothing has changed, never skipped as an optimisation, and never delayed
behind file, network or broker I/O.

So the order inside a tick is deliberate. Perception and decision-making happen
first and are allowed to be slow-ish; the send happens unconditionally; logging
happens after the send. If anything raises, the loop stops the robot and then
re-raises. An ``atexit`` handler and a signal handler do the same, so that an
unclean exit still stops the wheels.

Simulated and real runs differ only in a :class:`Clock` and a transport. That
is what makes it possible to test the loop -- watchdog timing included -- at a
thousand times real speed and still be testing the code that will drive the
robot.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Callable, Protocol

from .config import Config
from .control import Command
from .fsm import NavFsm, NavInputs, State
from .link.detections import DetectionSource
from .link.esp32 import Esp32Link, Telemetry
from .odom import Odometry, TelemetryCrossCheck
from .runlog import RunLog

__all__ = ["Clock", "RealClock", "SimClock", "ControlLoop", "LoopStats"]


class Clock(Protocol):
    def monotonic(self) -> float: ...
    def sleep_until(self, t: float) -> None: ...


class RealClock:
    def monotonic(self) -> float:
        return time.monotonic()

    def sleep_until(self, t: float) -> None:
        dt = t - time.monotonic()
        if dt > 0:
            time.sleep(dt)


class SimClock:
    """Virtual time. Advances only when the loop asks it to.

    ``pump`` is called with the elapsed slice so the simulated firmware and
    chassis advance in step with the loop. No wall-clock time passes, so a
    five-minute mission is a fraction of a second of test.
    """

    def __init__(self, pump: Callable[[float], None] | None = None, step: float = 0.005) -> None:
        self.t = 0.0
        self.pump = pump
        self.step = step

    def monotonic(self) -> float:
        return self.t

    def sleep_until(self, t: float) -> None:
        while self.t < t:
            dt = min(self.step, t - self.t)
            if self.pump is not None:
                self.pump(dt)
            self.t += dt


@dataclass
class LoopStats:
    ticks: int = 0
    overruns: int = 0
    worst_tick_ms: float = 0.0
    late_commands: int = 0  # sends that missed the watchdog margin
    stop_reason: str = ""

    def as_dict(self) -> dict:
        return dict(self.__dict__)


@dataclass
class ControlLoop:
    robot: Config
    nav: Config
    link: Esp32Link
    detections: DetectionSource
    fsm: NavFsm
    odom: Odometry
    clock: Clock = field(default_factory=RealClock)
    log: RunLog | None = None
    cross_check: TelemetryCrossCheck = field(default_factory=TelemetryCrossCheck)
    on_tick: Callable[[dict], None] | None = None

    def __post_init__(self) -> None:
        self.rate_hz = self.nav.get("loop.rate_hz")
        self.dt = self.nav.get("loop.dt_s")
        self.max_stall_ms = self.nav.get("loop.max_stall_ms")
        self.stats = LoopStats()
        self._magnet_on: bool | None = None
        self._stop = False
        # When a detection message last arrived, on *this loop's* clock. Not
        # the source's: the source stamps with time.monotonic(), which is the
        # wall clock, and under simulation the wall clock has no relationship
        # to the run. Measuring the heartbeat against the loop's own clock is
        # also the right thing on hardware -- it is the loop that must notice.
        self._last_frame_t: float | None = None

    def request_stop(self, reason: str = "asked to stop") -> None:
        self._stop = True
        self.stats.stop_reason = reason

    # -- one tick -------------------------------------------------------

    def tick(self) -> Command:
        now = self.clock.monotonic()

        frames = self.detections.poll()
        telemetry_list = self.link.poll()
        telemetry: Telemetry | None = telemetry_list[-1] if telemetry_list else None
        for t in telemetry_list:
            self.odom.update(t.ticks_l, t.ticks_r, t=t.t_ms / 1000.0)
        if telemetry is not None:
            complaint = self.cross_check.check(self.odom, telemetry.v_meas, telemetry.omega_meas)
            if complaint and self.log is not None:
                self.log.note(f"CROSS-CHECK: {complaint}")

        if frames:
            self._last_frame_t = now
        age = float("inf") if self._last_frame_t is None else now - self._last_frame_t
        cmd = self.fsm.update(
            NavInputs(
                now=now,
                odom_pose=self.odom.pose,
                frames=frames,
                vision_age_s=age,
                telemetry=telemetry,
            )
        )

        # The send. Unconditional, before any logging, every single tick.
        self.link.send_velocity(cmd.v, cmd.omega)

        want_magnet = self.fsm.magnet_should_be_on
        if want_magnet != self._magnet_on:
            self.link.magnet(want_magnet)
            self._magnet_on = want_magnet

        self._log_tick(now, cmd, telemetry, age, len(frames))
        return cmd

    def _log_tick(self, now, cmd, telemetry, vision_age, n_frames) -> None:
        if self.log is None and self.on_tick is None:
            return
        record = {
            "t": round(now, 4),
            "state": self.fsm.state.value,
            "v": round(cmd.v, 4),
            "omega": round(cmd.omega, 4),
            "reason": cmd.reason,
            "x": round(self.odom.pose.x, 4),
            "y": round(self.odom.pose.y, 4),
            "theta": round(self.odom.pose.theta, 4),
            "vision_age_s": None if math.isinf(vision_age) else round(vision_age, 4),
            "frames": n_frames,
        }
        if telemetry is not None:
            record["flags"] = f"{int(telemetry.flags):02x}"
            record["seq"] = telemetry.seq
            record["v_meas"] = telemetry.v_meas
        if self.log is not None:
            self.log.tick(record)
        if self.on_tick is not None:
            self.on_tick(record)

    # -- the loop -------------------------------------------------------

    def run(self, duration_s: float | None = None, until_done: bool = True) -> LoopStats:
        """Run until told to stop, the mission finishes, or time runs out.

        Always stops the wheels on the way out, including on the way out
        through an exception.
        """
        start = self.clock.monotonic()
        next_tick = start
        try:
            while not self._stop:
                tick_start = self.clock.monotonic()
                if duration_s is not None and tick_start - start >= duration_s:
                    self.stats.stop_reason = self.stats.stop_reason or "duration reached"
                    break

                cmd = self.tick()
                self.stats.ticks += 1

                if until_done and self.fsm.state is State.DONE:
                    self.stats.stop_reason = f"mission complete: {cmd.reason}"
                    break

                elapsed_ms = (self.clock.monotonic() - tick_start) * 1000.0
                self.stats.worst_tick_ms = max(self.stats.worst_tick_ms, elapsed_ms)
                if elapsed_ms > self.max_stall_ms:
                    self.stats.overruns += 1
                    if self.log is not None:
                        self.log.note(
                            f"tick took {elapsed_ms:.1f} ms, over the {self.max_stall_ms:.0f} ms "
                            f"budget -- the watchdog is in play"
                        )

                next_tick += self.dt
                if next_tick < self.clock.monotonic():
                    # Fell behind. Resync rather than sprinting to catch up:
                    # a burst of commands does not undo a late one.
                    next_tick = self.clock.monotonic() + self.dt
                self.clock.sleep_until(next_tick)
        except BaseException as e:
            # Any exception at all, including KeyboardInterrupt. Stop first,
            # ask questions afterwards (CLAUDE.md section 6).
            self.stats.stop_reason = f"{type(e).__name__}: {e}"
            self.safe_stop()
            raise
        finally:
            self.safe_stop()
        return self.stats

    def safe_stop(self) -> None:
        """``S``, and the drum off. Safe to call repeatedly, never raises."""
        try:
            self.link.magnet(False)
        except Exception:
            pass
        try:
            self.link.stop()
        except Exception:
            pass
