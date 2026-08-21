"""The waypoint controller and pure pursuit, open-loop geometry and in the sim."""

from __future__ import annotations

import math

import pytest

from fodnav.config import load_nav_config, load_robot_config
from fodnav.control import (
    Command,
    Gains,
    MotionLimits,
    PathFollower,
    WaypointController,
    find_lookahead_point,
    saturate,
    unicycle_to_wheels,
    wheels_to_unicycle,
)
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


@pytest.fixture(scope="module")
def gains(nav):
    return Gains.from_config(nav)


@pytest.fixture(scope="module")
def limits(robot):
    return MotionLimits.from_config(robot)


def drive(robot, ctrl, params=None, seconds=90.0, pose=Pose2D(), dt=DT):
    """Close the loop: controller -> sim -> odometry -> controller."""
    sim = UnicycleSim(robot, params or SimParams.perfect(), pose=pose)
    odo = Odometry.from_config(robot, pose=pose)
    odo.update(*sim.ticks)
    for i in range(int(seconds / dt)):
        cmd = ctrl.update(odo.pose)
        if cmd.done:
            return sim, odo, i * dt
        sim.step(cmd.v, cmd.omega, dt)
        odo.update(*sim.ticks, t=sim.t)
    return sim, odo, seconds


# -- saturation and the deadband -----------------------------------------


def test_commands_are_clamped_to_what_the_chassis_can_do():
    lim = MotionLimits(v_max=0.45, v_min=0.03, omega_max=3.0, omega_min=0.15)
    assert saturate(99.0, -99.0, lim) == (0.45, -3.0)
    assert saturate(-99.0, 99.0, lim) == (-0.45, 3.0)


def test_a_nonzero_command_is_lifted_out_of_the_friction_deadband():
    # Otherwise the controller asks for 8 mm/s at the end of an approach, the
    # robot does not move at all, and the loop keeps believing it is closing.
    lim = MotionLimits(v_max=0.45, v_min=0.03, omega_max=3.0, omega_min=0.15)
    assert saturate(0.008, 0.0, lim)[0] == pytest.approx(0.03)
    assert saturate(-0.008, 0.0, lim)[0] == pytest.approx(-0.03)


def test_a_spin_from_rest_is_lifted_to_the_minimum_spin_rate():
    lim = MotionLimits(v_max=0.45, v_min=0.03, omega_max=3.0, omega_min=0.15)
    assert saturate(0.0, 0.02, lim)[1] == pytest.approx(0.15)
    assert saturate(0.0, -0.02, lim)[1] == pytest.approx(-0.15)


def test_a_small_heading_correction_while_driving_is_left_alone():
    # omega_min is a stiction figure measured spinning in place from rest. It
    # is not a floor on turn rate while the wheels are already moving, and
    # applying it as one turns every small correction into a bang-bang shimmy
    # all the way down a coverage row.
    lim = MotionLimits(v_max=0.45, v_min=0.03, omega_max=3.0, omega_min=0.15)
    v, omega = saturate(0.27, 0.01, lim)
    assert omega == pytest.approx(0.01)
    assert v == pytest.approx(0.27)


def test_a_zero_command_stays_zero():
    # Lifting zero would mean a stopped robot creeps, and "stopped" has to mean
    # stopped -- V 0.000 0.000 is sent 50 times a second while parked.
    lim = MotionLimits(v_max=0.45, v_min=0.03, omega_max=3.0, omega_min=0.15)
    assert saturate(0.0, 0.0, lim) == (0.0, 0.0)


def test_wheel_conversion_round_trips():
    for v, omega in [(0.3, 0.0), (0.0, 1.0), (-0.2, 0.5)]:
        l, r = unicycle_to_wheels(v, omega, 0.2)
        assert wheels_to_unicycle(l, r, 0.2) == pytest.approx((v, omega))


def test_a_left_turn_spins_the_right_wheel_faster():
    # Positive omega is counter-clockwise (REP-103), so the right wheel leads.
    l, r = unicycle_to_wheels(0.0, 1.0, 0.2)
    assert r > 0 > l


# -- the waypoint controller, geometry -----------------------------------


def test_it_turns_before_driving_when_the_goal_is_behind(gains, limits):
    c = WaypointController((-1.0, 0.0), gains, limits)
    cmd = c.update(Pose2D(0, 0, 0))
    assert cmd.v == 0.0 and abs(cmd.omega) > 0
    assert c.phase == "turn"


def test_it_drives_straight_when_already_aimed(gains, limits):
    c = WaypointController((2.0, 0.0), gains, limits)
    cmd = c.update(Pose2D(0, 0, 0))
    assert cmd.v > 0
    assert cmd.omega == pytest.approx(0.0, abs=1e-9)


def test_it_declares_arrival_inside_the_goal_radius(gains, limits):
    c = WaypointController((1.0, 0.0), gains, limits)
    cmd = c.update(Pose2D(1.0 - gains.goal_radius_m / 2, 0, 0))
    assert cmd.done and cmd.is_stop


def test_arrival_latches(gains, limits):
    # Odometry drift must not un-arrive a waypoint and send the robot back.
    c = WaypointController((1.0, 0.0), gains, limits)
    c.update(Pose2D(1.0, 0.0, 0.0))
    assert c.update(Pose2D(0.0, 0.0, 0.0)).done


def test_forward_speed_falls_off_as_the_bearing_error_grows(gains, limits):
    c = WaypointController((2.0, 0.0), gains, limits)
    straight = c.update(Pose2D(0, 0, 0)).v
    c2 = WaypointController((2.0, 0.0), gains, limits)
    c2.phase = "drive"
    skewed = c2.update(Pose2D(0, 0, 0.4)).v
    assert 0 < skewed < straight


def test_it_stops_steering_on_the_last_few_centimetres(gains, limits):
    # At 5 cm range a 3 cm lateral offset is a 30-degree bearing error. A
    # controller that steers on it circles the waypoint instead of reaching it.
    c = WaypointController((1.0, 0.0), gains, limits)
    c.phase = "drive"
    cmd = c.update(Pose2D(1.0 - 1.5 * gains.goal_radius_m, 0.03, 0.0))
    assert cmd.omega == 0.0
    assert cmd.v > 0
    assert c.phase == "final"


def test_a_large_heading_error_mid_drive_stops_and_re_aims(gains, limits):
    c = WaypointController((2.0, 0.0), gains, limits)
    c.phase = "drive"
    cmd = c.update(Pose2D(0.0, 0.0, 2.0))
    assert cmd.v == 0.0 and c.phase == "turn"


def test_every_command_it_issues_is_achievable(gains, limits):
    # Nothing over the limits, and nothing that asks the chassis to start
    # moving slower than it can start moving, at any pose.
    c = WaypointController((2.0, 1.0), gains, limits)
    for x in [0.0, 0.5, 1.0, 1.9, 1.99]:
        for th in [-3.0, -1.0, 0.0, 0.5, 3.0]:
            cmd = c.update(Pose2D(x, 0.0, th))
            assert abs(cmd.v) <= limits.v_max and abs(cmd.omega) <= limits.omega_max
            assert cmd.v == 0.0 or abs(cmd.v) >= limits.v_min
            if cmd.v == 0.0 and cmd.omega != 0.0:
                # An in-place spin has to break stiction; a correction made
                # while already driving does not.
                assert abs(cmd.omega) >= limits.omega_min


# -- the waypoint controller, closed loop --------------------------------


@pytest.mark.parametrize("goal", [(2.0, 0.0), (2.0, 1.0), (-1.5, 0.5), (0.0, -2.0), (0.3, 0.3)])
def test_it_reaches_the_goal_on_a_perfect_robot(robot, gains, limits, goal):
    sim, odo, t = drive(robot, WaypointController(goal, gains, limits))
    assert math.hypot(goal[0] - odo.pose.x, goal[1] - odo.pose.y) <= gains.goal_radius_m
    assert t < 60.0


def test_it_reaches_a_goal_directly_behind_it(robot, gains, limits):
    sim, odo, t = drive(robot, WaypointController((-2.0, 0.0), gains, limits))
    assert math.hypot(-2.0 - odo.pose.x, odo.pose.y) <= gains.goal_radius_m


def test_it_does_not_orbit(robot, gains, limits):
    # The classic failure: bearing goes ill-conditioned near the goal and the
    # robot circles at goal_radius forever, always steering, never arriving.
    sim, odo, t = drive(robot, WaypointController((1.0, 0.0), gains, limits), seconds=30)
    assert t < 20.0, "it should arrive, not circle until the test times out"
    assert odo.rotation_rad < 4 * math.pi


def test_it_still_arrives_through_stiction_noise_and_slip(robot, gains, limits):
    p = SimParams(tick_noise_std=1.0, slip_prob=0.05, seed=3)
    sim, odo, t = drive(robot, WaypointController((2.0, 1.0), gains, limits), params=p)
    assert math.hypot(2.0 - odo.pose.x, 1.0 - odo.pose.y) <= gains.goal_radius_m


def test_odometry_arrives_but_the_robot_does_not(robot, gains, limits, nav):
    # The whole argument of CLAUDE.md §5, as a test. Over 4 m with a 1.5%
    # wheel-scale mismatch the robot is confident and wrong, and it is wrong by
    # far more than the drum is wide. This is why visual servoing exists and
    # why move_to is not the controller for the throw-a-nail demo.
    sim, odo, t = drive(
        robot, WaypointController((4.0, 0.0), gains, limits), params=SimParams.from_config(nav)
    )
    believed = math.hypot(4.0 - odo.pose.x, 0.0 - odo.pose.y)
    actual = math.hypot(4.0 - sim.true_pose.x, 0.0 - sim.true_pose.y)
    assert believed <= gains.goal_radius_m, "odometry is sure it arrived"
    assert actual > 0.25, "and it is nowhere near"
    assert actual > 2 * robot.get("drum.width_m"), "further off than the drum is wide"


# -- pure pursuit ---------------------------------------------------------


def test_the_lookahead_point_is_a_lookahead_away():
    path = [(0.0, 0.0), (5.0, 0.0)]
    pt, progress = find_lookahead_point(path, Pose2D(1.0, 0.0, 0.0), 0.3)
    assert pt == pytest.approx((1.3, 0.0))
    assert progress == pytest.approx(1.0)


def test_the_lookahead_clamps_to_the_end_of_the_path():
    # Past the end there is nothing to aim at, so it aims at the last point.
    path = [(0.0, 0.0), (1.0, 0.0)]
    pt, _ = find_lookahead_point(path, Pose2D(0.95, 0.0, 0.0), 0.3)
    assert pt == (1.0, 0.0)


def test_progress_cannot_snap_back_onto_the_row_just_finished():
    # A boustrophedon runs beside itself: rows are one spacing apart in space
    # but a whole row-length apart along the path. An unwindowed nearest-point
    # search decides the robot is back on the previous row the moment it drifts
    # half a spacing, and the sweep starts over.
    path = [(0.0, 0.0), (3.0, 0.0), (3.0, 0.15), (0.0, 0.15)]
    on_row_two = Pose2D(1.5, 0.15, math.pi)
    _, progress = find_lookahead_point(path, on_row_two, 0.3, progress_s=4.65)
    assert progress > 3.15, "progress must stay on the second row"


def test_progress_cannot_leap_forward_onto_a_row_not_yet_driven():
    # The other direction: with a 30 cm lookahead and 15 cm row spacing, a
    # circle-intersection formulation happily targets two rows ahead.
    path = [(0.0, 0.0), (3.0, 0.0), (3.0, 0.15), (0.0, 0.15), (0.0, 0.30), (3.0, 0.30)]
    _, progress = find_lookahead_point(path, Pose2D(1.0, 0.02, 0.0), 0.3, progress_s=1.0)
    assert progress < 3.0, "it must still be on the first row"


def test_the_lookahead_point_is_never_behind_the_robot():
    path = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0)]
    progress = None
    for x in [0.1, 0.5, 1.0, 1.5, 1.9, 1.99]:
        pose = Pose2D(x, 0.0, 0.0)
        pt, progress = find_lookahead_point(path, pose, 0.3, progress)
        ahead, _ = pose.inverse_transform_point(*pt)
        assert ahead > 0, f"lookahead {pt} is behind the robot at x={x}"


def test_an_empty_path_is_refused(gains, limits):
    with pytest.raises(ValueError):
        PathFollower([], gains, limits)


def test_it_converges_onto_a_line_it_starts_beside(robot, gains, limits):
    pf = PathFollower([(0.0, 0.0), (4.0, 0.0)], gains, limits)
    sim, odo, t = drive(robot, pf, pose=Pose2D(0.0, 0.30, 0.0))
    assert abs(pf.cross_track_error(odo.pose)) < 0.02


def test_it_follows_a_boustrophedon_without_cutting_the_corners(robot, gains, limits):
    path = [(0.0, 0.0), (3.0, 0.0), (3.0, 0.4), (0.0, 0.4), (0.0, 0.8), (3.0, 0.8)]
    pf = PathFollower(path, gains, limits)
    worst = 0.0
    sim = UnicycleSim(robot, SimParams.perfect())
    odo = Odometry.from_config(robot)
    odo.update(*sim.ticks)
    for _ in range(int(150 / DT)):
        cmd = pf.update(odo.pose)
        if cmd.done:
            break
        sim.step(cmd.v, cmd.omega, DT)
        odo.update(*sim.ticks, t=sim.t)
        worst = max(worst, abs(pf.cross_track_error(odo.pose)))
    assert pf.done, "it must finish the path"
    assert math.hypot(odo.pose.x - 3.0, odo.pose.y - 0.8) <= gains.goal_radius_m
    # The turns are tighter than the lookahead, so some excursion is expected;
    # what matters is that it stays inside the swath it is supposed to sweep.
    assert worst < robot.get("drum.width_m")


def test_cross_track_error_is_signed_to_the_left(gains, limits):
    pf = PathFollower([(0.0, 0.0), (4.0, 0.0)], gains, limits)
    assert pf.cross_track_error(Pose2D(1.0, 0.2, 0.0)) > 0
    assert pf.cross_track_error(Pose2D(1.0, -0.2, 0.0)) < 0


def test_a_finished_path_stays_finished(robot, gains, limits):
    pf = PathFollower([(0.0, 0.0), (1.0, 0.0)], gains, limits)
    drive(robot, pf)
    assert pf.update(Pose2D(0.0, 0.0, 0.0)).done
