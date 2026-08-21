"""Odometry: tick arithmetic, closed-form arcs, and the cross-check."""

from __future__ import annotations

import math

import pytest

from fodnav.config import MissingValueError, load_robot_config
from fodnav.frames import Pose2D
from fodnav.frames import wrap_angle
from fodnav.odom import Odometry, TelemetryCrossCheck, tick_delta

INT32_MAX = 2**31 - 1
INT32_MIN = -(2**31)

R, W, TPR = 0.0325, 0.20, 1440.0
MPT = 2.0 * math.pi * R / TPR  # metres per tick


def od(pose=Pose2D()):
    o = Odometry(R, W, TPR, pose)
    o.update(0, 0)  # seed
    return o


def drive(o, d_left_m, d_right_m, steps=200, t0=0.0, dt=0.02):
    """Feed cumulative counters for a constant-curvature move."""
    for i in range(1, steps + 1):
        o.update(
            round(d_left_m * i / steps / MPT),
            round(d_right_m * i / steps / MPT),
            t=t0 + i * dt,
        )
    return o.pose


# -- tick arithmetic ------------------------------------------------------


@pytest.mark.parametrize(
    "new, old, expect",
    [
        (5, 3, 2),
        (3, 5, -2),
        (0, 0, 0),
        (INT32_MIN, INT32_MAX, 1),          # forward across the wrap
        (INT32_MAX, INT32_MIN, -1),         # backward across the wrap
        (INT32_MIN + 99, INT32_MAX - 100, 200),
        (-1, 1, -2),
        (1, -1, 2),
    ],
)
def test_tick_delta_crosses_the_wrap(new, old, expect):
    assert tick_delta(new, old) == expect


def test_tick_delta_is_antisymmetric():
    for a, b in [(5, 3), (INT32_MIN, INT32_MAX), (0, 12345), (-77, 88)]:
        assert tick_delta(a, b) == -tick_delta(b, a)


def test_driving_across_the_wrap_boundary_changes_nothing():
    # The counters wrap roughly every fortnight at speed. If this is wrong, the
    # robot teleports several hundred kilometres, once, unreproducibly.
    near_wrap = od()
    near_wrap._last = (INT32_MAX - 500, INT32_MAX - 500)
    ordinary = od()
    ordinary._last = (0, 0)
    for i in range(1, 400):
        near_wrap.update(
            tick_wrap(INT32_MAX - 500 + i), tick_wrap(INT32_MAX - 500 + 2 * i)
        )
        ordinary.update(i, 2 * i)
    assert (near_wrap.pose.x, near_wrap.pose.y, near_wrap.pose.theta) == pytest.approx(
        (ordinary.pose.x, ordinary.pose.y, ordinary.pose.theta), abs=1e-12
    )


def tick_wrap(v: int) -> int:
    v &= 0xFFFFFFFF
    return v - 2**32 if v >= 2**31 else v


# -- integration ----------------------------------------------------------


def test_a_straight_line_is_a_straight_line():
    o = od()
    drive(o, 1.0, 1.0)
    assert (o.pose.x, o.pose.y, o.pose.theta) == pytest.approx((1.0, 0.0, 0.0), abs=1e-3)
    assert o.distance_m == pytest.approx(1.0, abs=1e-3)


def test_driving_backwards_goes_backwards():
    o = od()
    drive(o, -0.5, -0.5)
    assert o.pose.x == pytest.approx(-0.5, abs=1e-3)
    assert o.distance_m == pytest.approx(0.5, abs=1e-3), "path length is unsigned"


def test_an_in_place_spin_does_not_translate():
    o = od()
    arc = W / 2.0 * math.pi  # each wheel drives half a track-width times pi
    drive(o, -arc, arc)
    assert o.pose.theta == pytest.approx(math.pi, abs=1e-3)
    assert (o.pose.x, o.pose.y) == pytest.approx((0.0, 0.0), abs=1e-3)


@pytest.mark.parametrize("radius", [0.25, 0.5, 1.0, 2.0])
@pytest.mark.parametrize("sweep", [math.pi / 4, math.pi / 2, math.pi])
def test_a_known_arc_integrates_to_its_closed_form(radius, sweep):
    # A quarter circle of radius R ends at (R, R) facing 90 degrees. This is
    # the test that catches the Euler approximation, which lands short and
    # inside, systematically, every time.
    o = od()
    drive(o, (radius - W / 2) * sweep, (radius + W / 2) * sweep, steps=400)
    assert o.pose.x == pytest.approx(radius * math.sin(sweep), abs=2e-3)
    assert o.pose.y == pytest.approx(radius * (1 - math.cos(sweep)), abs=2e-3)
    # Compare wrapped: a half-circle ends exactly on the +/-pi seam, where
    # 3.1416 and -3.1416 are the same heading and only the difference is
    # meaningful.
    assert wrap_angle(o.pose.theta - sweep) == pytest.approx(0.0, abs=2e-3)


def test_a_single_large_step_still_integrates_the_arc_exactly():
    # One step, 90 degrees. Euler would put this badly wrong; the exact form
    # does not care about step size.
    o = od()
    radius, sweep = 0.5, math.pi / 2
    drive(o, (radius - W / 2) * sweep, (radius + W / 2) * sweep, steps=1)
    assert (o.pose.x, o.pose.y) == pytest.approx((0.5, 0.5), abs=1e-3)


def test_the_pose_does_not_depend_on_the_timestamps():
    # Telemetry arrives jittery. A late line must not move the robot on the map.
    a, b = od(), od()
    for i in range(1, 100):
        a.update(i, 2 * i, t=i * 0.02)
        b.update(i, 2 * i, t=i * 0.02 * (1 + 0.5 * (i % 3)))
    assert (a.pose.x, a.pose.y, a.pose.theta) == pytest.approx(
        (b.pose.x, b.pose.y, b.pose.theta), abs=1e-12
    )


def test_velocity_is_reported_when_timestamps_are_given():
    o = od()
    ticks_per_metre = 1.0 / MPT
    for i in range(1, 50):
        n = round(0.3 * 0.02 * i * ticks_per_metre)
        o.update(n, n, t=i * 0.02)
    assert o.v == pytest.approx(0.3, rel=0.02)
    assert o.omega == pytest.approx(0.0, abs=1e-3)


def test_the_first_update_only_seeds_the_baseline():
    o = Odometry(R, W, TPR)
    o.update(123456, -654321)
    assert o.pose == Pose2D()
    assert o.n_updates == 0


def test_reset_moves_the_estimate_without_inventing_motion():
    o = od()
    drive(o, 1.0, 1.0)
    o.reset(Pose2D(5.0, 5.0, 1.0))
    before = o.pose
    o.update(*o._last)  # no new ticks
    assert (o.pose.x, o.pose.y, o.pose.theta) == pytest.approx(
        (before.x, before.y, before.theta)
    )


def test_starting_from_a_non_zero_pose_composes():
    o = od(Pose2D(1.0, 2.0, math.pi / 2))
    drive(o, 0.5, 0.5)
    assert (o.pose.x, o.pose.y) == pytest.approx((1.0, 2.5), abs=1e-3)


@pytest.mark.parametrize("bad", [(0, W, TPR), (R, 0, TPR), (R, W, 0), (-R, W, TPR)])
def test_nonsense_geometry_is_refused(bad):
    with pytest.raises(ValueError):
        Odometry(*bad)


def test_odometry_refuses_to_build_on_an_uncalibrated_robot():
    with pytest.raises(MissingValueError, match="odometry"):
        Odometry.from_config(load_robot_config("config/robot.yaml"))


# -- the cross-check ------------------------------------------------------


def _agreeing_odom(v=0.3, omega=0.0):
    o = od()
    o.v, o.omega = v, omega
    return o


def test_agreement_is_silent():
    c = TelemetryCrossCheck()
    o = _agreeing_odom()
    for _ in range(200):
        assert c.check(o, 0.30, 0.0) is None


def test_a_stationary_robot_proves_nothing():
    c = TelemetryCrossCheck()
    o = _agreeing_odom(v=0.0)
    for _ in range(200):
        assert c.check(o, 0.0, 0.0) is None
    assert c.n_compared == 0


def test_an_inverted_encoder_is_named_as_such():
    # Mirrored odometry looks like a broken controller and is not. The message
    # must say where the fix goes: firmware, not the Pi.
    c = TelemetryCrossCheck(strikes=5)
    o = _agreeing_odom(v=0.30)
    msgs = [c.check(o, -0.30, 0.0) for _ in range(20)]
    complaints = [m for m in msgs if m]
    assert complaints and "sign" in complaints[0]
    assert "firmware" in complaints[0]


def test_a_wrong_tick_constant_is_caught():
    c = TelemetryCrossCheck(strikes=5)
    o = _agreeing_odom(v=0.30)
    complaints = [m for m in (c.check(o, 0.45, 0.0) for _ in range(20)) if m]
    assert complaints and "ticks_per_rev" in complaints[0]


def test_a_wrong_track_width_is_caught():
    c = TelemetryCrossCheck(strikes=5)
    o = _agreeing_odom(v=0.30, omega=1.0)
    complaints = [m for m in (c.check(o, 0.30, 0.6) for _ in range(20)) if m]
    assert complaints and "track_width" in complaints[0]


def test_one_bad_sample_does_not_cry_wolf():
    # A dropped line or a filter lag must not produce a warning, or everyone
    # learns to ignore the warnings.
    c = TelemetryCrossCheck(strikes=25)
    o = _agreeing_odom(v=0.30)
    for _ in range(10):
        assert c.check(o, 0.30, 0.0) is None
    assert c.check(o, 0.9, 0.0) is None
    for _ in range(10):
        assert c.check(o, 0.30, 0.0) is None
