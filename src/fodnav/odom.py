"""Differential-drive odometry, integrated on the Pi from raw encoder ticks.

The ESP32 sends **cumulative** tick counters, not deltas, and they wrap as
signed 32-bit (docs/protocol.md section 4). Cumulative because a dropped
telemetry line then costs nothing: the next line still carries the total. The
price is that the Pi must difference them and handle wraparound explicitly,
which is what :func:`tick_delta` is for and why it has its own tests.

Odometry is computed here and not taken from the firmware's ``v_meas`` /
``omega_meas``. Those exist so that a sign flip, a swapped encoder or a wrong
tick constant shows up early and loudly (:class:`TelemetryCrossCheck`) rather
than as a controller that mysteriously will not converge.

This is the drifting estimate. It is smooth and continuous, which is what a
controller needs, and it is wrong by a slowly growing amount, which is why
``move_to`` over 4 m is a different problem from visual servoing (CLAUDE.md
section 5).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .config import Config
from .frames import Pose2D, wrap_angle

__all__ = ["Odometry", "TelemetryCrossCheck", "tick_delta", "TICK_MODULUS"]

TICK_MODULUS = 1 << 32
_TICK_HALF = 1 << 31


def tick_delta(new: int, old: int) -> int:
    """Signed difference of two int32 counters, across the wrap.

    Correct as long as the true delta is under 2^31 ticks between readings,
    which at any speed this chassis can reach is a matter of weeks. The failure
    mode if it were not: a single 2-billion-tick jump, i.e. the robot deciding
    it has teleported several hundred kilometres.
    """
    d = (int(new) - int(old)) % TICK_MODULUS
    if d >= _TICK_HALF:
        d -= TICK_MODULUS
    return d


@dataclass
class Odometry:
    """Integrates wheel ticks into a pose in the ``odom`` frame.

    ``odom`` starts coincident with ``world`` and drifts from it. Nothing here
    tries to correct that drift; correcting it is what the visual servo does,
    by not depending on it at all.
    """

    wheel_radius_m: float
    track_width_m: float
    ticks_per_rev: float
    pose: Pose2D = Pose2D()

    def __post_init__(self) -> None:
        if self.wheel_radius_m <= 0 or self.track_width_m <= 0 or self.ticks_per_rev <= 0:
            raise ValueError("wheel radius, track width and ticks per rev must all be positive")
        self._metres_per_tick = 2.0 * math.pi * self.wheel_radius_m / self.ticks_per_rev
        self._last: tuple[int, int] | None = None
        self._last_t: float | None = None
        self.v = 0.0
        self.omega = 0.0
        self.distance_m = 0.0  # path length, not displacement
        self.rotation_rad = 0.0  # total absolute rotation
        self.n_updates = 0

    @classmethod
    def from_config(cls, robot: Config, pose: Pose2D = Pose2D()) -> "Odometry":
        r, w, tpr = robot.require(
            "drive.wheel_radius_m",
            "drive.track_width_m",
            "drive.ticks_per_rev",
            needed_by="odometry",
        )
        return cls(wheel_radius_m=r, track_width_m=w, ticks_per_rev=tpr, pose=pose)

    @property
    def metres_per_tick(self) -> float:
        return self._metres_per_tick

    def reset(self, pose: Pose2D = Pose2D(), *, keep_counters: bool = True) -> None:
        """Move the estimate without pretending the wheels jumped.

        ``keep_counters`` leaves the tick baseline alone, so the *next* update
        still measures a correct delta. Dropping it would inject one bogus step
        of motion at the moment of the reset.
        """
        self.pose = pose
        if not keep_counters:
            self._last = None
        self.v = self.omega = 0.0

    def update(self, ticks_l: int, ticks_r: int, t: float | None = None) -> Pose2D:
        """Fold one telemetry sample in. The first call only seeds the baseline.

        ``t`` is the *timestamp of the sample*, used solely to report ``v`` and
        ``omega``. The pose integration itself does not use time at all -- it
        is a function of tick deltas -- so a late or jittery telemetry line
        displaces nothing. That is a property worth keeping: a timing hiccup
        should not move the robot on the map.
        """
        if self._last is None:
            self._last = (int(ticks_l), int(ticks_r))
            self._last_t = t
            return self.pose

        dl_ticks = tick_delta(ticks_l, self._last[0])
        dr_ticks = tick_delta(ticks_r, self._last[1])
        self._last = (int(ticks_l), int(ticks_r))

        d_l = dl_ticks * self._metres_per_tick
        d_r = dr_ticks * self._metres_per_tick
        ds = 0.5 * (d_l + d_r)
        dtheta = (d_r - d_l) / self.track_width_m

        self.pose = _integrate(self.pose, ds, dtheta)
        self.distance_m += abs(ds)
        self.rotation_rad += abs(dtheta)
        self.n_updates += 1

        if t is not None and self._last_t is not None:
            dt = t - self._last_t
            if dt > 1e-9:
                self.v = ds / dt
                self.omega = dtheta / dt
        self._last_t = t
        return self.pose


def _integrate(pose: Pose2D, ds: float, dtheta: float) -> Pose2D:
    """Exact constant-curvature step.

    Not the Euler approximation ``x += ds*cos(theta)``. Over a 90-degree turn
    at any usable step size the difference is millimetres per step and
    centimetres per turn, all of it systematic and all of it in the same
    direction -- which is the kind of error that looks like a miscalibrated
    track width and gets "fixed" by mis-measuring one.
    """
    if abs(dtheta) < 1e-9:
        return Pose2D(
            pose.x + ds * math.cos(pose.theta),
            pose.y + ds * math.sin(pose.theta),
            pose.theta,
        )
    radius = ds / dtheta
    th1 = pose.theta + dtheta
    return Pose2D(
        pose.x + radius * (math.sin(th1) - math.sin(pose.theta)),
        pose.y - radius * (math.cos(th1) - math.cos(pose.theta)),
        th1,
    )


class TelemetryCrossCheck:
    """Compare the firmware's own velocity estimate with the Pi's.

    ``v_meas`` and ``omega_meas`` are not the odometry source. They are here so
    that the three failure modes which produce *confidently mirrored* odometry
    -- a swapped encoder pair, an inverted sign, a wrong ``ticks_per_rev`` --
    are caught in the first few seconds instead of being mistaken for a
    controller that needs more gain.

    Hysteresis is deliberate: a single disagreeing sample is a dropped line or
    a filter lag, and crying wolf about it would train everyone to ignore this.
    """

    def __init__(
        self,
        rel_tol: float = 0.25,
        abs_tol: float = 0.05,
        min_speed: float = 0.05,
        strikes: int = 25,
    ) -> None:
        self.rel_tol = rel_tol
        self.abs_tol = abs_tol
        self.min_speed = min_speed
        self.strikes = strikes
        self.v_strikes = 0
        self.omega_strikes = 0
        self.sign_strikes = 0
        self.n_compared = 0

    def check(self, odom: Odometry, v_meas: float, omega_meas: float) -> str | None:
        """Returns a complaint worth logging loudly, or ``None``."""
        if abs(odom.v) < self.min_speed and abs(v_meas) < self.min_speed:
            return None  # at rest everything agrees and nothing is proven
        self.n_compared += 1

        if odom.v * v_meas < 0 and min(abs(odom.v), abs(v_meas)) > self.min_speed:
            self.sign_strikes += 1
            if self.sign_strikes >= self.strikes:
                self.sign_strikes = 0
                return (
                    f"encoder sign disagreement: the Pi integrates v={odom.v:+.3f} m/s while "
                    f"the firmware reports v={v_meas:+.3f} m/s. An inverted or swapped encoder "
                    f"produces odometry that is confidently mirrored and the robot will drive "
                    f"away from every goal. Fix the sign in firmware (HARDWARE.md §2.4), not here."
                )
        else:
            self.sign_strikes = max(0, self.sign_strikes - 1)

        def disagrees(a: float, b: float) -> bool:
            return abs(a - b) > max(self.abs_tol, self.rel_tol * max(abs(a), abs(b)))

        if disagrees(odom.v, v_meas):
            self.v_strikes += 1
            if self.v_strikes >= self.strikes:
                self.v_strikes = 0
                return (
                    f"forward-speed disagreement: Pi {odom.v:+.3f} m/s vs firmware "
                    f"{v_meas:+.3f} m/s. Suspect drive.ticks_per_rev or drive.wheel_radius_m "
                    f"differing between config/robot.yaml and the firmware's compile-time "
                    f"constants -- the I handshake should have caught that, so check it ran."
                )
        else:
            self.v_strikes = max(0, self.v_strikes - 1)

        if disagrees(odom.omega, omega_meas):
            self.omega_strikes += 1
            if self.omega_strikes >= self.strikes:
                self.omega_strikes = 0
                return (
                    f"turn-rate disagreement: Pi {odom.omega:+.3f} rad/s vs firmware "
                    f"{omega_meas:+.3f} rad/s. Suspect drive.track_width_m, or left and right "
                    f"encoders swapped."
                )
        else:
            self.omega_strikes = max(0, self.omega_strikes - 1)
        return None
