"""The visual servo and the terminal blind leg. This is the demo.

The end-to-end tests here run the whole vision pipeline -- scene, camera,
message, parse, project, associate, servo, chassis -- with the odometry error
model switched on, because the entire claim being tested is that this
controller does not care about odometry drift.
"""

from __future__ import annotations

import json
import math

import pytest

from fodnav.config import load_nav_config, load_robot_config
from fodnav.control import Gains, MotionLimits, WaypointController
from fodnav.frames import Pose2D
from fodnav.ground import GroundPoint, GroundProjector
from fodnav.link.detections import parse_message, select_targets
from fodnav.odom import Odometry
from fodnav.servo import ServoGains, ServoGeometry, ServoState, VisualServo
from fodnav.sim.camera import SimCamera
from fodnav.sim.scene import SceneNoise, SimScene
from fodnav.sim.unicycle import SimParams, UnicycleSim
from fodnav.target import TargetTracker, TrackerParams

DT = 0.02
VISION_DT = 1.0 / 30.0


@pytest.fixture(scope="module")
def robot():
    return load_robot_config("config/sim_robot.yaml")


@pytest.fixture(scope="module")
def nav():
    return load_nav_config("config/nav.yaml")


@pytest.fixture(scope="module")
def geom(robot, nav):
    return ServoGeometry.from_config(robot, nav)


@pytest.fixture(scope="module")
def limits(robot):
    return MotionLimits.from_config(robot)


def make_servo(nav, limits, geom):
    return VisualServo(ServoGains.from_config(nav), limits, geom)


# -- geometry -------------------------------------------------------------


def test_the_blind_leg_is_the_near_limit_plus_the_drum_offset(robot, nav, geom):
    near = robot.get("camera.fov_near_limit_m")
    drum = robot.get("drum.offset_x_m")
    extra = nav.get("servo.blind_leg_extra_m")
    assert geom.blind_leg_m == pytest.approx(near - drum + extra)
    assert geom.blind_leg_m > near, "the drum is behind the axle, so it is further than the near limit"


def test_the_stop_range_is_derived_from_the_drum_not_typed_in(robot, geom):
    assert geom.stop_range_m == robot.get("drum.offset_x_m")


def test_the_geometry_refuses_to_build_on_an_unmeasured_robot(nav):
    from fodnav.config import MissingValueError

    with pytest.raises(MissingValueError, match="blind leg"):
        ServoGeometry.from_config(load_robot_config("config/robot.yaml"), nav)


# -- the control law ------------------------------------------------------


def test_it_turns_before_driving_at_a_target_off_to_the_side(nav, limits, geom):
    s = make_servo(nav, limits, geom)
    cmd = s.update(GroundPoint(0.5, 0.8), Pose2D(), 0.0)  # ~58 degrees off
    assert cmd.v == 0.0 and cmd.omega > 0


def test_it_steers_toward_the_target(nav, limits, geom):
    s = make_servo(nav, limits, geom)
    left = s.update(GroundPoint(1.0, 0.15), Pose2D(), 0.0)
    s.reset()
    right = s.update(GroundPoint(1.0, -0.15), Pose2D(), 0.0)
    assert left.omega > 0 > right.omega, "positive bearing is to the left (REP-103)"
    assert left.v > 0 and right.v > 0


def test_it_slows_down_as_it_closes(nav, limits, geom):
    s = make_servo(nav, limits, geom)
    far = s.update(GroundPoint(1.5, 0.0), Pose2D(), 0.0).v
    near = s.update(GroundPoint(0.4, 0.0), Pose2D(), 0.1).v
    assert 0 < near < far


def test_it_slows_down_when_badly_aimed(nav, limits, geom):
    s = make_servo(nav, limits, geom)
    straight = s.update(GroundPoint(1.0, 0.0), Pose2D(), 0.0).v
    skewed = s.update(GroundPoint(1.0, 0.3), Pose2D(), 0.1).v
    assert 0 < skewed < straight


def test_no_target_at_all_means_stop_not_wander(nav, limits, geom):
    s = make_servo(nav, limits, geom)
    cmd = s.update(None, Pose2D(), 0.0)
    assert cmd.is_stop and not cmd.done


def test_a_dropped_frame_coasts_rather_than_stuttering(nav, limits, geom):
    s = make_servo(nav, limits, geom)
    s.update(GroundPoint(1.0, 0.0), Pose2D(), 0.0)
    cmd = s.update(None, Pose2D(), 0.03)  # one missed frame at 30 Hz
    assert cmd.v > 0 and not cmd.done


def test_a_target_gone_for_too_long_is_lost(nav, limits, geom):
    s = make_servo(nav, limits, geom)
    s.update(GroundPoint(1.0, 0.0), Pose2D(), 0.0)
    cmd = s.update(None, Pose2D(), 5.0)
    assert cmd.done and s.state is ServoState.LOST and cmd.is_stop


# -- the blind leg --------------------------------------------------------


def test_it_latches_when_the_target_reaches_the_near_limit(nav, limits, geom):
    s = make_servo(nav, limits, geom)
    s.update(GroundPoint(1.0, 0.0), Pose2D(), 0.0)
    assert s.state is ServoState.SERVOING
    s.update(GroundPoint(geom.fov_near_limit_m - 0.01, 0.0), Pose2D(), 0.1)
    assert s.state is ServoState.BLIND_LEG


def test_the_blind_leg_drives_the_latched_distance_and_stops(nav, limits, geom):
    s = make_servo(nav, limits, geom)
    rng = geom.fov_near_limit_m
    s.update(GroundPoint(rng, 0.0), Pose2D(), 0.0)
    assert s.state is ServoState.BLIND_LEG
    expected = rng - geom.drum_offset_x_m + geom.blind_leg_extra_m

    # Walk the robot forward in 1 cm steps and see where it declares itself done.
    x = 0.0
    for i in range(200):
        cmd = s.update(None, Pose2D(x, 0.0, 0.0), 0.1 + i * DT)
        if cmd.done:
            break
        x += 0.01
    assert s.state is ServoState.ARRIVED
    assert x == pytest.approx(expected, abs=0.011)


def test_the_blind_leg_ignores_anything_the_camera_reports(nav, limits, geom):
    # At this range a detection is either a different object or a projection at
    # the edge of the calibrated patch. Steering on it would undo the latch.
    s = make_servo(nav, limits, geom)
    s.update(GroundPoint(geom.fov_near_limit_m, 0.0), Pose2D(), 0.0)
    cmd = s.update(GroundPoint(0.9, 0.9), Pose2D(0.01, 0.0, 0.0), 0.1)
    assert s.state is ServoState.BLIND_LEG
    assert abs(cmd.omega) < 0.2, "it must not swing toward a late detection"


def test_a_stalled_blind_leg_gives_up_rather_than_grinding_on(nav, limits, geom):
    s = make_servo(nav, limits, geom)
    s.update(GroundPoint(geom.fov_near_limit_m, 0.0), Pose2D(), 0.0)
    cmd = s.update(None, Pose2D(0.0, 0.0, 0.0), 100.0)  # never moved
    assert cmd.done and s.state is ServoState.LOST


def test_the_blind_leg_holds_the_latched_heading(nav, limits, geom):
    s = make_servo(nav, limits, geom)
    s.update(GroundPoint(geom.fov_near_limit_m, 0.0), Pose2D(0, 0, 0), 0.0)
    drifted = s.update(None, Pose2D(0.05, 0.0, 0.2), 0.1)  # yawed left
    assert drifted.omega < 0, "it should steer back toward the latched heading"


# -- end to end -----------------------------------------------------------


def run_demo(robot, nav, target_xy, params=None, noise=None, start=Pose2D(), seconds=40.0):
    """The whole pipeline: scene -> camera -> message -> project -> track -> servo."""
    cam = SimCamera(robot)
    proj = GroundProjector(cam.calibration(), max_valid_range_m=robot.get("camera.fov_far_limit_m"))
    scene = SimScene(cam, noise=noise or SceneNoise(seed=1))
    scene.add(*target_xy)
    sim = UnicycleSim(robot, params or SimParams.from_config(nav), pose=start)
    odo = Odometry.from_config(robot, pose=start)
    odo.update(*sim.ticks)
    tracker = TargetTracker(TrackerParams.from_config(nav))
    servo = VisualServo(ServoGains.from_config(nav), MotionLimits.from_config(robot),
                        ServoGeometry.from_config(robot, nav))

    t, next_frame = 0.0, 0.0
    cmd = None
    while t < seconds:
        if t >= next_frame:
            next_frame += VISION_DT
            frame = parse_message(json.dumps(scene.render(sim.true_pose, t)))
            proj.check_frame_size(frame.frame_size)
            observations = []
            for d in select_targets(
                frame,
                nav.get("detections.target_classes"),
                nav.get("detections.ignore_classes"),
                nav.get("detections.drop_conf"),
            ):
                g = proj.project_detection(d)
                if g is not None:
                    observations.append((g, d.conf, d.cls))
            tracker.update(observations, t, odo.pose)
        best = tracker.best()
        cmd = servo.update(best.predict_base(odo.pose) if best else None, odo.pose, t)
        if cmd.done:
            break
        sim.step(cmd.v, cmd.omega, DT)
        odo.update(*sim.ticks, t=sim.t)
        t += DT
    return sim, odo, servo, t


def drum_miss(robot, sim, target_xy):
    dx, dy = sim.true_pose.transform_point(robot.get("drum.offset_x_m"), 0.0)
    return math.hypot(dx - target_xy[0], dy - target_xy[1])


@pytest.mark.parametrize("target", [(1.2, 0.0), (1.3, 0.35), (1.1, -0.3), (0.8, 0.15)])
def test_the_drum_ends_up_over_the_target(robot, nav, target):
    # With the shipped odometry error model on. The servo re-measures every
    # frame, so drift does not accumulate into the answer.
    sim, odo, servo, t = run_demo(robot, nav, target)
    assert servo.state is ServoState.ARRIVED, f"ended in {servo.state}"
    assert drum_miss(robot, sim, target) < robot.get("drum.capture_width_m") / 2


def test_it_beats_dead_reckoning_to_the_same_point(robot, nav):
    # The comparison the whole of CLAUDE.md §5 rests on. Same target, same
    # chassis, same errors; one controller re-measures and the other does not.
    target = (1.3, 0.35)
    sim_servo, _, servo, _ = run_demo(robot, nav, target)
    servo_miss = drum_miss(robot, sim_servo, target)

    limits = MotionLimits.from_config(robot)
    gains = Gains.from_config(nav)
    ctrl = WaypointController(target, gains, limits)
    sim = UnicycleSim(robot, SimParams.from_config(nav))
    odo = Odometry.from_config(robot)
    odo.update(*sim.ticks)
    for _ in range(int(60 / DT)):
        cmd = ctrl.update(odo.pose)
        if cmd.done:
            break
        sim.step(cmd.v, cmd.omega, DT)
        odo.update(*sim.ticks, t=sim.t)
    blind_miss = math.hypot(sim.true_pose.x - target[0], sim.true_pose.y - target[1])

    assert servo_miss < blind_miss


def test_it_survives_a_detector_that_drops_frames_and_wobbles(robot, nav):
    noise = SceneNoise(conf=0.7, conf_noise=0.12, jitter_px=6.0, miss_rate=0.25, seed=5)
    sim, odo, servo, t = run_demo(robot, nav, (1.2, 0.2), noise=noise)
    assert servo.state is ServoState.ARRIVED
    assert drum_miss(robot, sim, (1.2, 0.2)) < robot.get("drum.width_m") / 2


def test_it_works_from_a_starting_pose_that_is_not_the_origin(robot, nav):
    start = Pose2D(2.0, -1.0, 1.2)
    dx, dy = start.transform_point(1.2, 0.1)
    sim, odo, servo, t = run_demo(robot, nav, (dx, dy), start=start)
    assert servo.state is ServoState.ARRIVED
    assert drum_miss(robot, sim, (dx, dy)) < robot.get("drum.capture_width_m") / 2


def test_the_target_really_does_vanish_before_the_drum_reaches_it(robot, nav):
    # If this ever stops being true the blind leg is unnecessary -- and if the
    # real fov_near_limit measurement comes back tiny, that is worth knowing.
    sim, odo, servo, t = run_demo(robot, nav, (1.2, 0.0))
    assert servo.latched_range_m > 0.0
    assert servo.latched_range_m <= robot.get("camera.fov_near_limit_m") + 0.02
