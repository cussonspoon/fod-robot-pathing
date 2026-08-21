"""The simulator, and specifically whether it reproduces the error that matters.

A simulator that says ``move_to`` works is worth nothing unless it also says
*why* the hardware will disagree. These tests pin the mechanism, not just the
magnitude.
"""

from __future__ import annotations

import math

import pytest

from fodnav.config import load_nav_config, load_robot_config
from fodnav.frames import Pose2D
from fodnav.odom import Odometry
from fodnav.sim.unicycle import SimParams, UnicycleSim

DT = 0.02


@pytest.fixture(scope="module")
def robot():
    return load_robot_config("config/sim_robot.yaml")


@pytest.fixture(scope="module")
def nav():
    return load_nav_config("config/nav.yaml")


def run(robot, params, v, omega, seconds, dt=DT, pose=Pose2D()):
    """Drive a command open-loop, integrating odometry alongside the truth."""
    sim = UnicycleSim(robot, params, pose=pose)
    odo = Odometry.from_config(robot, pose=pose)
    odo.update(*sim.ticks)
    for _ in range(int(round(seconds / dt))):
        sim.step(v, omega, dt)
        odo.update(*sim.ticks, t=sim.t)
    return sim, odo


def position_error(sim, odo):
    return math.hypot(sim.true_pose.x - odo.pose.x, sim.true_pose.y - odo.pose.y)


# -- with no errors, odometry is exact ------------------------------------


def test_a_perfect_robot_is_tracked_to_within_a_tick(robot):
    sim, odo = run(robot, SimParams.perfect(), 0.3, 0.0, 10.0)
    assert sim.true_pose.x == pytest.approx(3.0, abs=1e-6)
    assert position_error(sim, odo) < 1e-3  # bounded by tick quantisation, not drifting


def test_quantisation_error_stays_bounded_rather_than_accumulating(robot):
    errs = []
    for seconds in (2.0, 10.0, 30.0):
        sim, odo = run(robot, SimParams.perfect(), 0.3, 0.0, seconds)
        errs.append(position_error(sim, odo))
    assert max(errs) < 1e-3, "truncation defers sub-tick motion, it does not lose it"


def test_a_perfect_robot_drives_the_arc_it_was_asked_for(robot):
    # v = 0.3, omega = 0.6 is a 0.5 m radius circle. Quarter of it in 2.618 s.
    sim, odo = run(robot, SimParams.perfect(), 0.3, 0.6, math.pi / 2 / 0.6)
    assert (sim.true_pose.x, sim.true_pose.y) == pytest.approx((0.5, 0.5), abs=1e-3)
    assert position_error(sim, odo) < 2e-3


# -- the error that matters ----------------------------------------------


def test_a_wheel_scale_mismatch_curves_a_robot_that_believes_it_is_straight(robot):
    # 1.5% on one wheel. The encoders see a straight line because the wheels
    # turned equally; the floor sees an arc.
    p = SimParams(wheel_scale_left=1.015)
    sim, odo = run(robot, p, 0.3, 0.0, 4.0 / 0.3)
    assert odo.pose.y == pytest.approx(0.0, abs=1e-3), "odometry insists it went straight"
    assert odo.pose.theta == pytest.approx(0.0, abs=1e-3)
    assert abs(sim.true_pose.y) > 0.3, "the truth is a long way off the line"
    assert abs(sim.true_pose.theta) > math.radians(10)


def test_the_scale_error_bends_the_correct_way(robot):
    # A larger left wheel drives the robot to the right (clockwise, negative
    # yaw). If this ever inverts, something swapped a sign and the sim would
    # be teaching the opposite lesson.
    left_big, _ = run(robot, SimParams(wheel_scale_left=1.015), 0.3, 0.0, 5.0)
    right_big, _ = run(robot, SimParams(wheel_scale_right=1.015), 0.3, 0.0, 5.0)
    assert left_big.true_pose.theta < 0 < right_big.true_pose.theta


def test_the_scale_error_grows_with_distance_and_is_systematic(robot):
    p = SimParams(wheel_scale_left=1.015)
    errs = [position_error(*run(robot, p, 0.3, 0.0, d / 0.3)) for d in (1.0, 2.0, 4.0)]
    assert errs[0] < errs[1] < errs[2]
    assert errs[2] > 4 * errs[0], "quadratic-ish in distance, not linear noise"


def test_scale_mismatch_dominates_gaussian_tick_noise(robot, nav):
    # CLAUDE.md §10 asserts this and the sim has to agree, or the sim is not
    # modelling the thing it exists to model.
    noise_only = position_error(
        *run(robot, SimParams(tick_noise_std=nav.get("sim.tick_noise_std")), 0.3, 0.0, 4.0 / 0.3)
    )
    scale_only = position_error(*run(robot, SimParams(wheel_scale_left=1.015), 0.3, 0.0, 4.0 / 0.3))
    assert scale_only > 5 * noise_only


def test_a_heading_bias_is_invisible_to_the_encoders(robot):
    sim, odo = run(robot, SimParams(heading_bias_radps=0.02), 0.3, 0.0, 10.0)
    assert odo.pose.theta == pytest.approx(0.0, abs=1e-6)
    assert sim.true_pose.theta == pytest.approx(0.2, rel=0.05)


def test_a_parked_robot_does_not_drift_under_the_heading_bias(robot):
    sim, _ = run(robot, SimParams(heading_bias_radps=0.05), 0.0, 0.0, 10.0)
    assert sim.true_pose.theta == 0.0
    assert sim.distance_m == 0.0


def test_slip_makes_odometry_over_report_distance(robot):
    sim, odo = run(robot, SimParams(slip_prob=0.3, slip_fraction=0.5, seed=7), 0.3, 0.0, 10.0)
    assert sim.n_slips > 0
    assert odo.distance_m > sim.distance_m * 1.02


# -- the friction deadband ------------------------------------------------


def test_a_command_below_the_deadband_does_not_start_the_robot(robot):
    # The failure this reproduces: a position controller creeps toward a
    # waypoint at a speed too small to overcome static friction, the robot sits
    # still, and the loop believes it is still approaching.
    sim, _ = run(robot, SimParams(), 0.01, 0.0, 3.0)
    assert sim.distance_m == 0.0


def test_the_deadband_releases_once_the_robot_is_already_moving(robot):
    sim = UnicycleSim(robot, SimParams())
    for _ in range(50):
        sim.step(0.10, 0.0, DT)
    moved = sim.distance_m
    for _ in range(50):
        sim.step(0.01, 0.0, DT)
    assert sim.distance_m > moved + 0.005


def test_a_spin_at_the_measured_minimum_rate_does_spin(robot):
    sim, _ = run(robot, SimParams(), 0.0, robot.get("drive.omega_min_radps"), 2.0)
    assert abs(sim.true_pose.theta) > 0.3


# -- encoders -------------------------------------------------------------


def test_the_counters_wrap_like_int32(robot):
    sim = UnicycleSim(robot, SimParams.perfect())
    sim.seed_ticks(2**31 - 50, 2**31 - 50)
    odo = Odometry.from_config(robot)
    odo.update(*sim.ticks)
    for _ in range(200):
        sim.step(0.3, 0.0, DT)
        odo.update(*sim.ticks)
    assert sim.ticks_l < 0, "the counter must actually have wrapped for this to prove anything"
    assert odo.pose.x == pytest.approx(sim.true_pose.x, abs=1e-3)


def test_ticks_are_always_in_range(robot):
    sim = UnicycleSim(robot, SimParams.perfect())
    sim.seed_ticks(2**31 - 10, -(2**31) + 10)
    for _ in range(500):
        sim.step(0.4, 0.5, DT)
        assert -(2**31) <= sim.ticks_l <= 2**31 - 1
        assert -(2**31) <= sim.ticks_r <= 2**31 - 1


def test_reversing_unwinds_the_counters(robot):
    sim = UnicycleSim(robot, SimParams.perfect())
    for _ in range(100):
        sim.step(0.3, 0.0, DT)
    forward = sim.ticks_l
    for _ in range(100):
        sim.step(-0.3, 0.0, DT)
    assert sim.ticks_l < forward
    assert sim.true_pose.x == pytest.approx(0.0, abs=1e-6)


# -- reproducibility ------------------------------------------------------


def test_the_same_seed_gives_the_same_run(robot):
    p = SimParams(tick_noise_std=1.0, slip_prob=0.1, seed=42)
    a, _ = run(robot, p, 0.3, 0.2, 5.0)
    b, _ = run(robot, p, 0.3, 0.2, 5.0)
    assert (a.true_pose.x, a.true_pose.y, a.ticks_l) == (b.true_pose.x, b.true_pose.y, b.ticks_l)


def test_different_seeds_give_different_runs(robot):
    a, _ = run(robot, SimParams(tick_noise_std=1.0, seed=1), 0.3, 0.0, 5.0)
    b, _ = run(robot, SimParams(tick_noise_std=1.0, seed=2), 0.3, 0.0, 5.0)
    assert a.ticks_l != b.ticks_l


def test_the_shipped_sim_config_actually_has_the_error_switched_on(nav):
    # If someone sets this to 1.0 to make a test pass, the sim stops being able
    # to tell them anything about a 4 m drive.
    assert nav.get("sim.wheel_scale_left") != nav.get("sim.wheel_scale_right")


def test_a_zero_or_negative_step_is_refused(robot):
    sim = UnicycleSim(robot, SimParams())
    with pytest.raises(ValueError):
        sim.step(0.1, 0.0, 0.0)
