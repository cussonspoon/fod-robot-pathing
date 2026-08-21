"""Visual servo: drive to a *detected* object, and finish the leg it cannot see.

This is the controller for the advisor's throw-a-nail demo, and it is not an
alternative implementation of ``move_to`` (CLAUDE.md section 5). It re-measures
the target every frame at 30 Hz, so **it is immune to odometry drift entirely**
-- which matters because a 4 m dead-reckoned drive on differential wheels over
shop concrete accumulates heading error, and heading error is exactly what
makes the robot arrive next to the nail instead of on it. "Throw again, robot
follows" requires continuous re-targeting anyway.

The control law is the forty lines CLAUDE.md describes: project the detection
into ``base``, take ``bearing = atan2(y, x)`` and ``range = hypot(x, y)``,
drive ``omega = k_omega * bearing`` and ``v = k_v * (range - stop_range)``, with
``v`` scaled down as ``|bearing|`` grows so the robot does not sprint while
turning.

The rest of this file is the terminal blind leg, which is the part that decides
whether the demo works.

**The target leaves the camera's field of view before the drum reaches it.**
The camera looks a fixed distance ahead; the drum is under or behind the axle.
So the servo cannot run to contact -- at some range the target simply stops
being in any frame, and no amount of gain fixes that. Handled explicitly here:
when ``range`` drops below ``fov_near_limit`` (a measured value in
``robot.yaml``), latch the last good ``base``-frame estimate and drive the
remaining distance plus the drum offset open-loop on odometry. It is a short
leg -- 30 cm on the simulated geometry -- so drift over it is negligible. What
is not negligible is discovering the problem on demo day.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum

from .config import Config
from .control import Command, MotionLimits, saturate
from .frames import Pose2D, wrap_angle
from .ground import GroundPoint

__all__ = ["ServoState", "ServoGains", "ServoGeometry", "VisualServo"]


class ServoState(Enum):
    IDLE = "idle"
    SERVOING = "servoing"
    BLIND_LEG = "blind_leg"
    ARRIVED = "arrived"
    LOST = "lost"


@dataclass(frozen=True)
class ServoGains:
    k_v: float
    k_omega: float
    turn_first_rad: float
    slowdown_bearing_rad: float
    lost_timeout_s: float
    blind_leg_timeout_s: float
    blind_leg_min_v_frac: float
    cruise_fraction: float

    @classmethod
    def from_config(cls, nav: Config) -> "ServoGains":
        return cls(
            k_v=nav.get("servo.k_v"),
            k_omega=nav.get("servo.k_omega"),
            turn_first_rad=nav.get("servo.turn_first_rad"),
            slowdown_bearing_rad=nav.get("control.slowdown_bearing_rad"),
            lost_timeout_s=nav.get("servo.lost_timeout_s"),
            blind_leg_timeout_s=nav.get("servo.blind_leg_timeout_s"),
            blind_leg_min_v_frac=nav.get("servo.blind_leg_min_v_frac"),
            cruise_fraction=nav.get("control.cruise_fraction"),
        )


@dataclass(frozen=True)
class ServoGeometry:
    """Where the camera stops seeing and where the drum actually is.

    Both measured. ``stop_range_m`` is *derived* from the drum offset rather
    than being a third number someone types in: the target should end up at the
    drum, so the range the servo is driving toward is the drum's own x position
    in ``base``. On this chassis the drum is behind the axle, so that number is
    negative and the servo keeps pulling forward right through the handover.
    """

    fov_near_limit_m: float
    drum_offset_x_m: float
    blind_leg_extra_m: float = 0.0

    @property
    def stop_range_m(self) -> float:
        return self.drum_offset_x_m

    @property
    def blind_leg_m(self) -> float:
        """How far the robot drives after the target disappears."""
        return self.fov_near_limit_m - self.drum_offset_x_m + self.blind_leg_extra_m

    @classmethod
    def from_config(cls, robot: Config, nav: Config | None = None) -> "ServoGeometry":
        near, drum = robot.require(
            "camera.fov_near_limit_m",
            "drum.offset_x_m",
            needed_by="the terminal blind leg",
        )
        extra = nav.get("servo.blind_leg_extra_m") if nav is not None else 0.0
        return cls(fov_near_limit_m=near, drum_offset_x_m=drum, blind_leg_extra_m=extra)


class VisualServo:
    """Bearing-and-range servo with an explicit open-loop finish.

    Feed it a target in ``base`` (or ``None`` when there is no target this
    frame) plus the current odometry pose, every control cycle. Odometry is
    used for exactly one thing -- measuring the length of the blind leg -- and
    nothing else here depends on it.
    """

    def __init__(
        self,
        gains: ServoGains,
        limits: MotionLimits,
        geometry: ServoGeometry,
    ) -> None:
        self.gains = gains
        self.limits = limits
        self.geom = geometry
        self.state = ServoState.IDLE
        self.last_target: GroundPoint | None = None
        self.last_seen_t: float | None = None
        self._blind_start_pose: Pose2D | None = None
        self._blind_start_t: float | None = None
        self._blind_distance_m = 0.0
        self._blind_heading = 0.0
        self.latched_range_m = 0.0

    # -- lifecycle ------------------------------------------------------

    def reset(self) -> None:
        self.state = ServoState.IDLE
        self.last_target = None
        self.last_seen_t = None
        self._blind_start_pose = None
        self._blind_start_t = None

    @property
    def finished(self) -> bool:
        return self.state in (ServoState.ARRIVED, ServoState.LOST)

    # -- the loop -------------------------------------------------------

    def update(
        self, target: GroundPoint | None, odom_pose: Pose2D, now: float
    ) -> Command:
        if self.state is ServoState.ARRIVED:
            return Command(0.0, 0.0, done=True, reason="arrived")
        if self.state is ServoState.BLIND_LEG:
            # Deliberately ignores ``target``. Anything the camera reports at
            # this range is either a different object or a projection near the
            # edge of the calibrated patch, and steering on it now would undo
            # the whole point of latching.
            return self._blind_leg(odom_pose, now)
        if target is None:
            return self._no_target(now)

        self.last_target = target
        self.last_seen_t = now
        self.state = ServoState.SERVOING

        bearing = wrap_angle(target.bearing_rad)
        rng = target.range_m

        if rng <= self.geom.fov_near_limit_m:
            return self._latch(target, odom_pose, now)

        if abs(bearing) > self.gains.turn_first_rad:
            # Too far off to drive at: turn first. Sprinting sideways at a
            # target 40 degrees off the nose is how the drum misses it.
            _, omega = saturate(0.0, self.gains.k_omega * bearing, self.limits)
            return Command(0.0, omega, reason=f"turning to target, {math.degrees(bearing):+.1f} deg")

        cruise = self.gains.cruise_fraction * self.limits.v_max
        v_want = min(cruise, self.gains.k_v * (rng - self.geom.stop_range_m))
        v_want *= _bearing_scale(bearing, self.gains.slowdown_bearing_rad)
        v, omega = saturate(v_want, self.gains.k_omega * bearing, self.limits)
        return Command(v, omega, reason=f"servoing, {rng:.3f} m, {math.degrees(bearing):+.1f} deg")

    # -- the pieces -----------------------------------------------------

    def _no_target(self, now: float) -> Command:
        if self.state is not ServoState.SERVOING or self.last_seen_t is None:
            return Command(0.0, 0.0, reason="no target")
        gone = now - self.last_seen_t
        if gone > self.gains.lost_timeout_s:
            self.state = ServoState.LOST
            return Command(0.0, 0.0, done=True, reason=f"target lost for {gone:.2f} s")
        # A dropped frame or two is normal at 30 Hz. Coast straight rather than
        # stopping dead, so a flicker does not turn into a stutter.
        v, _ = saturate(0.5 * self.gains.cruise_fraction * self.limits.v_max, 0.0, self.limits)
        return Command(v, 0.0, reason=f"target missing {gone:.2f} s, coasting")

    def _latch(self, target: GroundPoint, odom_pose: Pose2D, now: float) -> Command:
        """Freeze the last good estimate and hand over to odometry."""
        self.state = ServoState.BLIND_LEG
        self.latched_range_m = target.range_m
        self._blind_start_pose = odom_pose
        self._blind_start_t = now
        # Everything past here is measured, not guessed: the range we last saw,
        # plus however far behind the axle the drum sits.
        self._blind_distance_m = max(
            0.0,
            target.range_m - self.geom.drum_offset_x_m + self.geom.blind_leg_extra_m,
        )
        self._blind_heading = wrap_angle(odom_pose.theta + target.bearing_rad)
        return self._blind_leg(odom_pose, now)

    def _blind_leg(self, odom_pose: Pose2D, now: float) -> Command:
        assert self._blind_start_pose is not None and self._blind_start_t is not None
        travelled = math.hypot(
            odom_pose.x - self._blind_start_pose.x, odom_pose.y - self._blind_start_pose.y
        )
        remaining = self._blind_distance_m - travelled
        if remaining <= 0.0:
            self.state = ServoState.ARRIVED
            return Command(0.0, 0.0, done=True, reason=f"drum over target after {travelled:.3f} m")

        if now - self._blind_start_t > self.gains.blind_leg_timeout_s:
            # Something is wrong -- a stall, a slip, a wheel off the ground.
            # Stop rather than grinding forward on an estimate that is now old.
            self.state = ServoState.LOST
            return Command(0.0, 0.0, done=True, reason="blind leg timed out")

        # Hold the latched heading. This is still open-loop with respect to
        # vision, which is what "blind" means; using odometry to keep straight
        # over 30 cm is free.
        heading_err = wrap_angle(self._blind_heading - odom_pose.theta)
        cruise = self.gains.cruise_fraction * self.limits.v_max
        # The same control law as the servoing phase, with the range coming
        # from odometry instead of from the camera. Using a different law here
        # would put a speed discontinuity exactly at the handover -- the robot
        # would decelerate toward the target, then jump back to cruise the
        # instant it stopped being able to see it.
        # ...with a floor. A proportional law converges exponentially, so
        # without one the last few centimetres take longer than the whole rest
        # of the leg and the timeout fires on a robot that was doing fine. The
        # drum is centimetres wide; that precision was never worth the seconds.
        floor = self.gains.blind_leg_min_v_frac * cruise
        v_want = min(cruise, max(floor, self.gains.k_v * remaining))
        v, omega = saturate(v_want, self.gains.k_omega * heading_err, self.limits)
        return Command(v, omega, reason=f"blind leg, {remaining:.3f} m to go")


def _bearing_scale(bearing: float, slowdown_bearing_rad: float) -> float:
    if abs(bearing) >= slowdown_bearing_rad:
        return 0.0
    return math.cos(0.5 * math.pi * bearing / slowdown_bearing_rad)
