"""A fake ESP32: docs/protocol.md, a watchdog, and a chassis to drive.

Two jobs.

First, it makes the whole stack runnable and testable with no hardware. The
control loop encodes a ``V`` line, this decodes it, clamps it, drives
:class:`~fodnav.sim.unicycle.UnicycleSim` with it and emits a ``T`` line back --
so the codec, the framing, the wraparound arithmetic, the flags and the
handshake are all exercised by every simulated run rather than by inspection.

Second, it is a **reference implementation of the protocol** that Teemy can
read alongside the document. Where the document is ambiguous, this is what nav
assumed; where this and the document disagree, the document wins and this is
the bug.

It is deliberately unfaithful in one direction only: time is explicit. Nothing
here sleeps. :meth:`FakeFirmware.step` advances an internal clock by whatever
you pass it, which makes the watchdog testable in microseconds rather than in
real 300 ms waits.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from ..config import Config
from ..frames import Pose2D
from ..link.esp32 import (
    WATCHDOG_FEEDING,
    WATCHDOG_TIMEOUT_MS,
    CmdMagnet,
    CmdSimple,
    CmdVelocity,
    Esp32Link,
    Flags,
    Info,
    LineReader,
    LoopbackTransport,
    ProtocolError,
    Telemetry,
    encode_info,
    encode_log,
    encode_telemetry,
    parse_command,
)
from .unicycle import SimParams, UnicycleSim

__all__ = ["FakeFirmware", "FirmwareConstants", "build_sim_link"]


@dataclass
class FirmwareConstants:
    """What the firmware was *compiled* with.

    Separate from ``config/robot.yaml`` on purpose. In the real system these
    live in Teemy's source and the ``I`` handshake asserts the two agree; being
    able to set them apart here is what lets a test prove the handshake
    actually catches a divergence instead of merely claiming to.
    """

    ticks_per_rev: float
    wheel_radius_m: float
    track_width_m: float
    v_max_mps: float
    omega_max_radps: float
    fw_version: str = "sim-0.1.0"

    @classmethod
    def from_config(cls, robot: Config, **over) -> "FirmwareConstants":
        tpr, r, w, vmax, wmax = robot.require(
            "drive.ticks_per_rev",
            "drive.wheel_radius_m",
            "drive.track_width_m",
            "drive.v_max_mps",
            "drive.omega_max_radps",
            needed_by="the simulated firmware",
        )
        kw = dict(
            ticks_per_rev=tpr, wheel_radius_m=r, track_width_m=w,
            v_max_mps=vmax, omega_max_radps=wmax,
        )
        kw.update(over)
        return cls(**kw)


class FakeFirmware:
    """The ESP32 side of the link. Executes velocities and reports encoders.

    It does not navigate, and it must never learn to: any feature that would
    give the firmware an opinion about where the robot should go belongs on the
    Pi (docs/protocol.md section 8).
    """

    def __init__(
        self,
        robot: Config,
        sim: UnicycleSim | None = None,
        transport: LoopbackTransport | None = None,
        constants: FirmwareConstants | None = None,
        telemetry_rate_hz: float = 50.0,
        proto_version: int = 1,
    ) -> None:
        self.robot = robot
        self.sim = sim if sim is not None else UnicycleSim(robot, SimParams())
        self.transport = transport if transport is not None else LoopbackTransport()
        self.k = constants or FirmwareConstants.from_config(robot)
        self.proto_version = proto_version
        self.telemetry_period = 1.0 / telemetry_rate_hz

        self.t = 0.0
        self.reader = LineReader()
        self.seq = 0
        self._next_telemetry = 0.0

        # state
        self.enabled = False
        self.magnet_on = False
        self.v_cmd = 0.0
        self.omega_cmd = 0.0
        self.watchdog_fired = False
        self.obstacle = False
        self.battery_low = False
        self.driver_fault = False
        self.fault_state = False
        self._clamped_this_cycle = False
        self.t_last_command: float | None = None  # firmware clock at the last watchdog-feeding line

        # counters, for tests and for the run log
        self.n_commands = 0
        self.n_velocity_commands = 0
        self.n_malformed = 0
        self.n_unknown = 0
        self.n_watchdog_trips = 0
        self.t_watchdog_fired: float | None = None  # firmware clock at the trip

    # -- flags ----------------------------------------------------------

    @property
    def flags(self) -> Flags:
        f = 0
        if self.enabled:
            f |= Flags.MOTORS_ENABLED
        if self.watchdog_fired:
            f |= Flags.WATCHDOG_FIRED
        if self.driver_fault:
            f |= Flags.DRIVER_FAULT
        if self.obstacle:
            f |= Flags.OBSTACLE
        if self._clamped_this_cycle:
            f |= Flags.CLAMPED
        if self.battery_low:
            f |= Flags.BATTERY_LOW
        if self.fault_state:
            f |= Flags.FAULT_STATE
        return Flags(f)

    @property
    def ms_since_command(self) -> float:
        if self.t_last_command is None:
            return float("inf")
        return (self.t - self.t_last_command) * 1000.0

    # -- the loop -------------------------------------------------------

    def step(self, dt: float) -> None:
        """Advance the firmware and the chassis by ``dt`` seconds."""
        self._consume_input()
        self._run_watchdog()

        v, omega = (self.v_cmd, self.omega_cmd) if self._may_move() else (0.0, 0.0)
        self.sim.step(v, omega, dt)
        self.t += dt

        while self.t >= self._next_telemetry:
            self._emit_telemetry()
            self._next_telemetry += self.telemetry_period

    def _may_move(self) -> bool:
        return self.enabled and not self.watchdog_fired and not self.fault_state

    def _run_watchdog(self) -> None:
        """Section 6. The single most important requirement in the protocol.

        Recovery needs an explicit ``E``: commands returning is not evidence
        that whatever stopped them has been fixed, and resuming into a stored
        0.5 m/s command after a two-second gap is precisely the failure the
        watchdog exists to prevent.
        """
        if self.watchdog_fired:
            return
        if self.t_last_command is None:
            # Not armed yet. The firmware has been powered on but the Pi has
            # never spoken, so there is nothing to protect against and the
            # motors are disabled anyway. Arming at boot instead would leave
            # bit 1 set from power-on and make it useless as a diagnostic.
            # docs/protocol.md §6 says this explicitly so that both sides do it.
            return
        if self.ms_since_command > WATCHDOG_TIMEOUT_MS:
            self.watchdog_fired = True
            self.n_watchdog_trips += 1
            self.t_watchdog_fired = self.t
            self.v_cmd = self.omega_cmd = 0.0
            self._log("E", f"watchdog: no command for {WATCHDOG_TIMEOUT_MS} ms, braking")

    def _consume_input(self) -> None:
        data = self.transport.device_read()
        if not data:
            return
        for raw in self.reader.feed(data):
            try:
                cmd = parse_command(raw)
            except ProtocolError as e:
                # Section 2: drop the whole line, log it, never act on part.
                self.n_malformed += 1
                self._log("W", f"malformed: {e}")
                continue
            if cmd is None:
                self.n_unknown += 1
                continue
            self.n_commands += 1
            self._apply(cmd, raw)

    def _apply(self, cmd, raw: bytes) -> None:
        tag = raw.decode("ascii", "replace").split(" ")[0]
        if tag in WATCHDOG_FEEDING:
            self.t_last_command = self.t

        if isinstance(cmd, CmdVelocity):
            self.n_velocity_commands += 1
            # Clamp, never reject: a rejected command in a 50 Hz loop becomes a
            # watchdog trip and a sudden stop.
            v = _clamp(cmd.v, self.k.v_max_mps)
            omega = _clamp(cmd.omega, self.k.omega_max_radps)
            self._clamped_this_cycle = (v != cmd.v) or (omega != cmd.omega)
            # Accepted and stored even while disabled, so the Pi can keep the
            # watchdog fed through a pause without the robot moving.
            self.v_cmd, self.omega_cmd = v, omega
        elif isinstance(cmd, CmdMagnet):
            self.magnet_on = cmd.on
        elif isinstance(cmd, CmdSimple):
            if cmd.tag == "S":
                self.v_cmd = self.omega_cmd = 0.0
            elif cmd.tag == "E":
                self.enabled = True
                self.watchdog_fired = False  # explicit recovery, as specified
            elif cmd.tag == "D":
                self.enabled = False
                self.v_cmd = self.omega_cmd = 0.0
            elif cmd.tag == "?":
                self._emit_telemetry()
            elif cmd.tag == "H":
                self.transport.device_write(
                    encode_info(
                        Info(
                            proto_version=self.proto_version,
                            fw_version=self.k.fw_version,
                            ticks_per_rev=self.k.ticks_per_rev,
                            wheel_radius_m=self.k.wheel_radius_m,
                            track_width_m=self.k.track_width_m,
                        )
                    )
                )

    def _emit_telemetry(self) -> None:
        t = Telemetry(
            seq=self.seq & 0xFFFF,
            t_ms=int(self.t * 1000.0) & 0xFFFFFFFF,
            ticks_l=self.sim.ticks_l,
            ticks_r=self.sim.ticks_r,
            # The firmware's own estimate, from its own compiled constants.
            # Not the Pi's odometry, and not the ground truth -- which is what
            # makes the cross-check able to catch a constants divergence.
            v_meas=self._measured_v(),
            omega_meas=self._measured_omega(),
            flags=self.flags,
        )
        self.transport.device_write(encode_telemetry(t))
        self.seq = (self.seq + 1) & 0xFFFF
        self._clamped_this_cycle = False

    def _measured_v(self) -> float:
        scale = self.k.wheel_radius_m / self.sim.wheel_radius_m
        return self.sim.v_true * scale if self._may_move() else 0.0

    def _measured_omega(self) -> float:
        scale = (self.k.wheel_radius_m / self.sim.wheel_radius_m) * (
            self.sim.track_width_m / self.k.track_width_m
        )
        return self.sim.omega_true * scale if self._may_move() else 0.0

    def _log(self, level: str, message: str) -> None:
        self.transport.device_write(encode_log(level, message))

    # -- fault injection, for tests -------------------------------------

    def set_obstacle(self, blocked: bool) -> None:
        self.obstacle = blocked

    def set_battery_low(self, low: bool) -> None:
        self.battery_low = low

    def latch_driver_fault(self) -> None:
        self.driver_fault = True
        self.fault_state = True
        self.v_cmd = self.omega_cmd = 0.0
        self._log("E", "driver fault latched, motion refused")


def _clamp(v: float, limit: float) -> float:
    return max(-abs(limit), min(abs(limit), v))


def build_sim_link(
    robot: Config,
    params: SimParams | None = None,
    pose: Pose2D = Pose2D(),
    constants: FirmwareConstants | None = None,
    on_log=None,
) -> tuple[Esp32Link, FakeFirmware]:
    """A link and a firmware wired to each other, for sim runs and tests.

    The link is the same :class:`~fodnav.link.esp32.Esp32Link` a real run uses,
    over the same codec. Only the transport differs.
    """
    transport = LoopbackTransport()
    sim = UnicycleSim(robot, params or SimParams(), pose=pose)
    fw = FakeFirmware(robot, sim=sim, transport=transport, constants=constants)
    link = Esp32Link(transport, robot=robot, on_log=on_log)
    return link, fw
