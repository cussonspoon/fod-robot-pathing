"""Velocity controllers for planned waypoints, and the wheel conversion.

This is the ``move_to(x, y)`` half of CLAUDE.md section 5. It integrates
odometry and accepts the drift, because a coverage waypoint is a position in
``world`` with no visual feature to servo on. The other half -- driving to a
*detected* object, which is immune to drift because it re-measures every frame
-- is :mod:`fodnav.servo`. They are not two implementations of the same thing
and using the wrong one for the wrong job is the main way this demo fails.

Everything here is a pure function of (pose, goal, limits) except for the small
amount of state needed for hysteresis, which lives in the controller objects.
Nothing here touches a serial port, a clock or a config file at call time.

The friction deadband gets first-class treatment. ``drive.v_min_mps`` is the
lowest speed that produces reliable motion, and a controller that does not know
about it will command 8 mm/s at the end of an approach, achieve nothing, and
keep believing it is still approaching. See :func:`saturate`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .config import Config
from .frames import Pose2D, wrap_angle

__all__ = [
    "MotionLimits",
    "Gains",
    "Command",
    "saturate",
    "unicycle_to_wheels",
    "wheels_to_unicycle",
    "WaypointController",
    "PathFollower",
    "find_lookahead_point",
    "closest_point_on_path",
    "cumulative_lengths",
    "point_at_arclength",
]


@dataclass(frozen=True)
class MotionLimits:
    """What the chassis can actually do. Measured, from ``robot.yaml``."""

    v_max: float
    v_min: float
    omega_max: float
    omega_min: float

    @classmethod
    def from_config(cls, robot: Config) -> "MotionLimits":
        v_max, v_min, w_max, w_min = robot.require(
            "drive.v_max_mps",
            "drive.v_min_mps",
            "drive.omega_max_radps",
            "drive.omega_min_radps",
            needed_by="the motion controllers",
        )
        return cls(v_max=v_max, v_min=v_min, omega_max=w_max, omega_min=w_min)

    @classmethod
    def unlimited(cls) -> "MotionLimits":
        """For unit-testing controller geometry in isolation. Never for a robot."""
        return cls(v_max=1e6, v_min=0.0, omega_max=1e6, omega_min=0.0)


@dataclass(frozen=True)
class Gains:
    """Tuning, from ``nav.yaml``. None of this is a fact about the hardware."""

    k_v: float
    k_omega: float
    goal_radius_m: float
    heading_tolerance_rad: float
    turn_in_place_rad: float
    cruise_fraction: float
    lookahead_m: float
    slowdown_bearing_rad: float

    @classmethod
    def from_config(cls, nav: Config) -> "Gains":
        return cls(
            k_v=nav.get("control.k_v"),
            k_omega=nav.get("control.k_omega"),
            goal_radius_m=nav.get("control.goal_radius_m"),
            heading_tolerance_rad=nav.get("control.heading_tolerance_rad"),
            turn_in_place_rad=nav.get("control.turn_in_place_rad"),
            cruise_fraction=nav.get("control.cruise_fraction"),
            lookahead_m=nav.get("control.lookahead_m"),
            slowdown_bearing_rad=nav.get("control.slowdown_bearing_rad"),
        )


@dataclass(frozen=True)
class Command:
    """What a controller wants the wheels to do, plus why.

    ``reason`` exists so the run log can answer "what was it thinking" without
    anyone having to reproduce the moment.
    """

    v: float = 0.0
    omega: float = 0.0
    done: bool = False
    reason: str = ""

    @property
    def is_stop(self) -> bool:
        return self.v == 0.0 and self.omega == 0.0


def saturate(v: float, omega: float, limits: MotionLimits) -> tuple[float, float]:
    """Clamp to what the chassis can do, and lift out of the deadband.

    Two different jobs, and the second is the one that is usually missed.

    *Clamping* to ``v_max``/``omega_max`` matches what the firmware does anyway
    (it saturates rather than rejecting, so that a big command does not become
    a watchdog trip), but doing it here too keeps the commanded and executed
    values equal, which keeps the run log honest.

    *Lifting* is the deadband: a nonzero command below ``v_min`` produces no
    motion at all, so it is raised to ``v_min`` rather than sent as a lie. The
    caller is responsible for deciding it wants to move at all -- a controller
    that asks for 1 mm/s when it should have declared itself finished will get
    ``v_min`` and overshoot, which is correct behaviour for a wrong question.

    **``omega_min`` is lifted only when the robot is otherwise stationary.**
    HARDWARE.md §2.5 measures it as the lowest reliable *in-place spin rate* --
    a stiction figure, taken from rest. It is not a floor on turn rate while
    driving: once both wheels are turning, an arbitrarily small difference
    between them is achievable, because there is no static friction left to
    break. Applying it as a floor anyway makes every small heading correction
    bang-bang between +/-omega_min, which on a real chassis is a visible shimmy
    down the whole length of a coverage row.
    """
    v = max(-limits.v_max, min(limits.v_max, v))
    omega = max(-limits.omega_max, min(limits.omega_max, omega))
    if 0.0 < abs(v) < limits.v_min:
        v = math.copysign(limits.v_min, v)
    if v == 0.0 and 0.0 < abs(omega) < limits.omega_min:
        omega = math.copysign(limits.omega_min, omega)
    return v, omega


def unicycle_to_wheels(v: float, omega: float, track_width_m: float) -> tuple[float, float]:
    """(v, omega) -> (left, right) wheel ground speeds, m/s.

    Nav never sends these -- the firmware owns the wheel loop and the protocol
    carries (v, omega). This is here for the simulator, for plots, and for
    checking that a command is achievable before sending it.
    """
    half = 0.5 * track_width_m
    return (v - omega * half, v + omega * half)


def wheels_to_unicycle(v_l: float, v_r: float, track_width_m: float) -> tuple[float, float]:
    return (0.5 * (v_l + v_r), (v_r - v_l) / track_width_m)


# ---------------------------------------------------------------------------
# move_to
# ---------------------------------------------------------------------------


class WaypointController:
    """Drive to a position in ``world``. Turn to face, then close on it.

    Three phases and a small amount of hysteresis:

    ``turn``     the goal is off to the side by more than ``turn_in_place_rad``,
                 so translating would just describe a lazy arc. Spin on the spot.
    ``drive``    steer with a heading P term while closing, with forward speed
                 scaled down as the bearing error grows so the robot does not
                 sprint sideways.
    ``final``    inside ``final_approach_m`` the bearing to the goal becomes
                 ill-conditioned -- a 3 cm offset at 5 cm range is a 30-degree
                 error, and steering on it makes the robot circle the waypoint
                 forever. So heading is frozen and the last few centimetres are
                 driven straight.

    That last phase is the difference between a controller that arrives and one
    that orbits, and it costs four lines.
    """

    def __init__(
        self,
        goal: tuple[float, float],
        gains: Gains,
        limits: MotionLimits,
        final_approach_factor: float = 3.0,
    ) -> None:
        self.goal = (float(goal[0]), float(goal[1]))
        self.gains = gains
        self.limits = limits
        self.final_approach_m = final_approach_factor * gains.goal_radius_m
        self.phase = "turn"
        self.done = False
        self._cruise = gains.cruise_fraction * limits.v_max

    def distance_to(self, pose: Pose2D) -> float:
        return math.hypot(self.goal[0] - pose.x, self.goal[1] - pose.y)

    def bearing_from(self, pose: Pose2D) -> float:
        """Heading error to the goal, in ``(-pi, pi]``, from the robot's frame."""
        dx, dy = self.goal[0] - pose.x, self.goal[1] - pose.y
        return wrap_angle(math.atan2(dy, dx) - pose.theta)

    def update(self, pose: Pose2D) -> Command:
        d = self.distance_to(pose)
        if d <= self.gains.goal_radius_m or self.done:
            self.done = True
            self.phase = "arrived"
            return Command(0.0, 0.0, done=True, reason=f"arrived, {d:.3f} m from goal")

        bearing = self.bearing_from(pose)

        if self.phase == "final" or d <= self.final_approach_m:
            # Bearing is ill-conditioned this close. Drive straight and let the
            # goal radius decide when it is over.
            self.phase = "final"
            v, omega = saturate(min(self._cruise, self.gains.k_v * d), 0.0, self.limits)
            return Command(v, omega, reason=f"final approach, {d:.3f} m")

        if self.phase == "turn":
            if abs(bearing) > self.gains.heading_tolerance_rad:
                _, omega = saturate(0.0, self.gains.k_omega * bearing, self.limits)
                return Command(0.0, omega, reason=f"turning to face, {math.degrees(bearing):+.1f} deg")
            self.phase = "drive"

        if abs(bearing) > self.gains.turn_in_place_rad:
            # Lost the heading badly -- a corner, a slip, a new goal. Stop and
            # re-aim rather than describing a long arc back onto the line.
            self.phase = "turn"
            _, omega = saturate(0.0, self.gains.k_omega * bearing, self.limits)
            return Command(0.0, omega, reason=f"re-aiming, {math.degrees(bearing):+.1f} deg off")

        v_want = min(self._cruise, self.gains.k_v * d) * _bearing_scale(
            bearing, self.gains.slowdown_bearing_rad
        )
        v, omega = saturate(v_want, self.gains.k_omega * bearing, self.limits)
        return Command(v, omega, reason=f"driving, {d:.3f} m to go")


def _bearing_scale(bearing: float, slowdown_bearing_rad: float) -> float:
    """Scale forward speed down as heading error grows, to zero at the limit.

    Cosine rather than a linear ramp: it is flat near zero, so a well-aimed
    robot is not slowed for nothing, and it falls off where it matters.
    """
    if abs(bearing) >= slowdown_bearing_rad:
        return 0.0
    return math.cos(0.5 * math.pi * bearing / slowdown_bearing_rad)


# ---------------------------------------------------------------------------
# pure pursuit
# ---------------------------------------------------------------------------


def cumulative_lengths(path: list[tuple[float, float]]) -> list[float]:
    """Arc length at each waypoint. ``cum[i]`` is the distance to ``path[i]``."""
    cum = [0.0]
    for a, b in zip(path, path[1:]):
        cum.append(cum[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
    return cum


def point_at_arclength(
    path: list[tuple[float, float]], cum: list[float], s: float
) -> tuple[tuple[float, float], int]:
    """The point ``s`` metres along the polyline, clamped to its ends."""
    if s <= 0.0:
        return path[0], 0
    if s >= cum[-1]:
        return path[-1], max(0, len(path) - 2)
    for i in range(len(path) - 1):
        if cum[i + 1] >= s:
            seg = cum[i + 1] - cum[i]
            t = 0.0 if seg <= 1e-12 else (s - cum[i]) / seg
            return (
                (path[i][0] + t * (path[i + 1][0] - path[i][0]),
                 path[i][1] + t * (path[i + 1][1] - path[i][1])),
                i,
            )
    return path[-1], max(0, len(path) - 2)


def closest_point_on_path(
    path: list[tuple[float, float]],
    point: tuple[float, float],
    s_min: float = 0.0,
    s_max: float | None = None,
    cum: list[float] | None = None,
) -> tuple[tuple[float, float], int, float]:
    """Nearest point on the polyline, restricted to an arc-length window.

    Returns ``(point, segment_index, arc_length)``. The window is what makes
    this usable on a path that runs beside itself: a boustrophedon's rows are
    one row-spacing apart in space but a whole row-length apart along the path,
    so an unrestricted nearest-point search will happily decide the robot is on
    the next row the moment it drifts half a spacing.
    """
    cum = cumulative_lengths(path) if cum is None else cum
    total = cum[-1]
    lo = max(0.0, min(s_min, total))
    hi = total if s_max is None else max(lo, min(s_max, total))

    best_d = float("inf")
    best: tuple[tuple[float, float], int, float] = (path[0], 0, 0.0)
    for i in range(len(path) - 1):
        seg = cum[i + 1] - cum[i]
        if seg <= 1e-12 or cum[i + 1] < lo or cum[i] > hi:
            continue
        a, b = path[i], path[i + 1]
        dx, dy = b[0] - a[0], b[1] - a[1]
        t = ((point[0] - a[0]) * dx + (point[1] - a[1]) * dy) / (seg * seg)
        # Clamp to the part of this segment that lies inside the window, not
        # just to the segment: a single coverage row is many windows long.
        t = max((lo - cum[i]) / seg, min((hi - cum[i]) / seg, t))
        t = max(0.0, min(1.0, t))
        cx, cy = a[0] + t * dx, a[1] + t * dy
        d = math.hypot(point[0] - cx, point[1] - cy)
        if d < best_d:
            best_d, best = d, ((cx, cy), i, cum[i] + t * seg)
    return best


def find_lookahead_point(
    path: list[tuple[float, float]],
    pose: Pose2D,
    lookahead_m: float,
    progress_s: float | None = None,
    back_m: float = 0.25,
    ahead_m: float | None = None,
) -> tuple[tuple[float, float], float]:
    """The point ``lookahead_m`` further along the path than the robot is.

    Arc length, not circle intersections, and the difference matters here.
    The textbook formulation -- take the furthest point where a circle of
    radius ``lookahead_m`` crosses the path -- breaks on a boustrophedon: row
    spacing is around 15 cm and a sensible lookahead is 30 cm, so that circle
    also crosses the next two rows and the "furthest" crossing is two rows
    ahead. The robot skips rows, or cuts one long diagonal across the arena and
    declares the sweep complete.

    Progress is carried as an arc length and the search is windowed around it,
    so the robot can advance smoothly but cannot teleport forward onto a row it
    has not driven, nor snap back onto the one it just finished.

    ``progress_s`` of ``None`` means "no prior progress": acquire by searching
    the whole path. That is right exactly once, at the start of a run, and
    wrong every tick afterwards.

    Returns the target point and the new progress, which the caller feeds back.
    """
    if not path:
        raise ValueError("cannot follow an empty path")
    if len(path) == 1:
        return path[0], 0.0

    cum = cumulative_lengths(path)
    if progress_s is None:
        lo, hi = 0.0, cum[-1]
    else:
        window = 3.0 * lookahead_m if ahead_m is None else ahead_m
        lo, hi = progress_s - back_m, progress_s + window
    _pt, _i, s = closest_point_on_path(path, (pose.x, pose.y), lo, hi, cum)
    target, _seg = point_at_arclength(path, cum, s + lookahead_m)
    return target, s


class PathFollower:
    """Pure pursuit over a polyline, for the long straight legs of a sweep.

    Steers toward a point a fixed distance ahead on the path. The curvature
    that gets there is ``2 y_L / L^2`` where ``y_L`` is the lookahead point's
    offset in the robot's own frame -- which is why the whole controller is a
    handful of lines and why it is smooth where a heading P-controller on the
    nearest point chatters.

    The lookahead distance is the entire tuning story. Too short and it weaves;
    too long and it cuts corners. Tune it in the simulator, on a path with real
    turns in it, before touching the robot.
    """

    def __init__(
        self,
        path: list[tuple[float, float]],
        gains: Gains,
        limits: MotionLimits,
    ) -> None:
        if not path:
            raise ValueError("cannot follow an empty path")
        self.path = [(float(x), float(y)) for x, y in path]
        self.gains = gains
        self.limits = limits
        self.cum = cumulative_lengths(self.path)
        # None until the first update, which acquires against the whole path.
        # After that it only ever moves within a window, so the robot cannot
        # skip a row it has not driven.
        self.progress_s: float | None = None
        self.done = False
        self._cruise = gains.cruise_fraction * limits.v_max
        self._final = WaypointController(self.path[-1], gains, limits)

    @property
    def length_m(self) -> float:
        return self.cum[-1]

    @property
    def remaining_m(self) -> float:
        return self.cum[-1] - (self.progress_s or 0.0)

    @property
    def fraction_done(self) -> float:
        if self.cum[-1] <= 0:
            return 0.0
        return min(1.0, (self.progress_s or 0.0) / self.cum[-1])

    def cross_track_error(self, pose: Pose2D) -> float:
        """Signed distance from the path, positive to the left of it.

        Not used by the controller -- it is the number that says whether the
        controller is working, which is a different job.
        """
        (cx, cy), i, _s = closest_point_on_path(self.path, (pose.x, pose.y), cum=self.cum)
        a, b = self.path[i], self.path[i + 1]
        d = math.hypot(pose.x - cx, pose.y - cy)
        return math.copysign(
            d, (b[0] - a[0]) * (pose.y - a[1]) - (b[1] - a[1]) * (pose.x - a[0])
        )

    def update(self, pose: Pose2D) -> Command:
        if self.done:
            return Command(0.0, 0.0, done=True, reason="path complete")

        # The last waypoint is handled by the waypoint controller, which knows
        # how to arrive. Pure pursuit does not: it has no notion of stopping.
        # Only once the sweep has actually progressed to the end, though -- a
        # coverage path passes near its own last waypoint on the way round, and
        # finishing there would end the run most of a sweep early.
        near_end = self.remaining_m <= max(self.gains.lookahead_m, self.gains.goal_radius_m)
        if near_end and math.hypot(
            self.path[-1][0] - pose.x, self.path[-1][1] - pose.y
        ) <= max(self.gains.lookahead_m, self.gains.goal_radius_m):
            cmd = self._final.update(pose)
            self.done = cmd.done
            return Command(cmd.v, cmd.omega, cmd.done, reason=f"end of path: {cmd.reason}")

        target, self.progress_s = find_lookahead_point(
            self.path, pose, self.gains.lookahead_m, self.progress_s
        )
        # The lookahead point in the robot's own frame. y is all that matters.
        _, y_l = pose.inverse_transform_point(*target)
        bearing = wrap_angle(math.atan2(target[1] - pose.y, target[0] - pose.x) - pose.theta)

        if abs(bearing) > self.gains.turn_in_place_rad:
            _, omega = saturate(0.0, self.gains.k_omega * bearing, self.limits)
            return Command(0.0, omega, reason=f"re-aiming onto path, {math.degrees(bearing):+.1f} deg")

        v = self._cruise * _bearing_scale(bearing, self.gains.slowdown_bearing_rad)
        curvature = 2.0 * y_l / (self.gains.lookahead_m**2)
        v, omega = saturate(v, v * curvature, self.limits)
        return Command(
            v, omega, reason=f"pursuing, {self.progress_s:.2f}/{self.cum[-1]:.2f} m along"
        )
