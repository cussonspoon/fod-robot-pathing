"""The state machine: the safety rules first, then the two paradigms."""

from __future__ import annotations

import pytest

from fodnav.config import load_nav_config, load_robot_config
from fodnav.frames import Pose2D
from fodnav.fsm import Mode, NavFsm, NavInputs, State
from fodnav.ground import GroundPoint, GroundProjector
from fodnav.link.esp32 import Flags, Telemetry
from fodnav.sim.camera import SimCamera


@pytest.fixture(scope="module")
def robot():
    return load_robot_config("config/sim_robot.yaml")


def nav_cfg(**overrides):
    cfg = load_nav_config("config/nav.yaml")
    for k, v in overrides.items():
        cfg._values[k.replace("__", ".")] = v
    return cfg


@pytest.fixture(scope="module")
def projector(robot):
    cam = SimCamera(robot)
    return GroundProjector(cam.calibration(), max_valid_range_m=robot.get("camera.fov_far_limit_m"))


def telem(flags: int = 0x01) -> Telemetry:
    return Telemetry(1, 1000, 0, 0, 0.0, 0.0, Flags(flags))


def alive(now=0.0, **kw):
    kw.setdefault("vision_age_s", 0.0)
    kw.setdefault("odom_pose", Pose2D())
    return NavInputs(now=now, **kw)


# -- the safety rules come first -----------------------------------------


def test_vision_silence_stops_the_robot(robot):
    fsm = NavFsm(robot, nav_cfg())
    cmd = fsm.update(alive(vision_age_s=99.0))
    assert cmd.is_stop and fsm.state is State.STOPPED
    assert "vision" in cmd.reason


def test_vision_that_has_never_spoken_also_stops_it(robot):
    # At startup "the camera has not started yet" and "the camera died" are the
    # same fact from here, and the safe response to both is not to move.
    fsm = NavFsm(robot, nav_cfg())
    cmd = fsm.update(NavInputs(now=0.0, odom_pose=Pose2D()))
    assert cmd.is_stop and "never" in cmd.reason


def test_an_empty_scene_is_not_silence(robot, projector):
    # An empty dets list means the floor is clear. Only the absence of messages
    # means vision is dead.
    fsm = NavFsm(robot, nav_cfg(), projector=projector)
    cmd = fsm.update(alive(vision_age_s=0.0))
    assert not cmd.is_stop or fsm.state is not State.STOPPED


def test_a_firmware_fault_stops_the_robot(robot):
    fsm = NavFsm(robot, nav_cfg())
    cmd = fsm.update(alive(telemetry=telem(0x80)))
    assert cmd.is_stop and "fault" in cmd.reason


def test_a_watchdog_trip_stops_the_robot(robot):
    # Recovery needs an explicit re-enable, so nav must not just keep driving.
    fsm = NavFsm(robot, nav_cfg())
    cmd = fsm.update(alive(telemetry=telem(0x03)))
    assert cmd.is_stop and "watchdog" in cmd.reason


def test_the_obstacle_flag_stops_the_robot_when_configured(robot):
    fsm = NavFsm(robot, nav_cfg(safety__stop_on_obstacle=True))
    assert fsm.update(alive(telemetry=telem(0x09))).is_stop
    fsm2 = NavFsm(robot, nav_cfg(safety__stop_on_obstacle=False))
    assert not fsm2.update(alive(telemetry=telem(0x09))).is_stop


def test_a_low_battery_is_a_warning_by_default(robot):
    # Bit 6 is a warning; the firmware owns the actual cutoff.
    fsm = NavFsm(robot, nav_cfg())
    assert not fsm.update(alive(telemetry=telem(0x41))).is_stop


def test_safety_beats_the_mission(robot):
    fsm = NavFsm(robot, nav_cfg(mission__mode="coverage"))
    fsm.update(alive())  # gets going
    assert fsm.update(alive(now=1.0, telemetry=telem(0x80))).is_stop


# -- the mode switch ------------------------------------------------------


def test_both_paradigms_are_reachable_from_config(robot):
    # CLAUDE.md §11: unresolved, so neither may be hardcoded and neither branch
    # may be deleted to tidy up.
    assert NavFsm(robot, nav_cfg(mission__mode="coverage")).mode is Mode.COVERAGE
    assert NavFsm(robot, nav_cfg(mission__mode="target")).mode is Mode.TARGET
    assert NavFsm(robot, nav_cfg(mission__mode="idle")).mode is Mode.IDLE


def test_idle_mode_holds_still(robot):
    fsm = NavFsm(robot, nav_cfg(mission__mode="idle"))
    assert fsm.update(alive()).is_stop


def test_coverage_mode_plans_a_path_up_front(robot):
    fsm = NavFsm(robot, nav_cfg(mission__mode="coverage"))
    assert len(fsm.path) >= 2
    assert fsm.swath_source == "drum"
    assert fsm.status()["row_spacing_m"] > 0


def test_the_coverage_plan_records_which_swath_it_used(robot):
    # The two readings of "coverage" give sweeps that differ by a factor of
    # three, so a run that does not say which it used cannot be compared.
    drum = NavFsm(robot, nav_cfg(mission__mode="coverage", planner__swath_source="drum"))
    cam = NavFsm(robot, nav_cfg(mission__mode="coverage", planner__swath_source="camera"))
    assert drum.status()["swath_source"] == "drum"
    assert cam.status()["swath_source"] == "camera"
    assert len(cam.path) < len(drum.path)


def test_the_arena_is_inset_before_planning(robot):
    fsm = NavFsm(robot, nav_cfg(mission__mode="coverage"))
    margin = robot.get("chassis.planner_margin_m")
    for x, y in fsm.path:
        assert x >= fsm.arena.x_min + margin - 1e-9
        assert x <= fsm.arena.x_max - margin + 1e-9


def test_coverage_drives_and_then_finishes(robot):
    fsm = NavFsm(robot, nav_cfg(mission__mode="coverage"))
    cmd = fsm.update(alive())
    assert fsm.state is State.COVER and not cmd.is_stop
    # Teleport to the end of the path; it should declare the mission done.
    fsm.follower.progress_s = fsm.follower.length_m
    end = fsm.path[-1]
    for i in range(5):
        cmd = fsm.update(alive(now=1.0 + i, odom_pose=Pose2D(end[0], end[1], 0.0)))
    assert fsm.state is State.DONE


# -- target mode ----------------------------------------------------------


def test_target_mode_holds_when_there_is_nothing_to_chase(robot, projector):
    fsm = NavFsm(robot, nav_cfg(mission__mode="target"), projector=projector)
    cmd = fsm.update(alive())
    assert cmd.is_stop and fsm.state is State.SEARCH
    assert "no target" in cmd.reason


def test_scan_mode_turns_on_the_spot_instead(robot, projector):
    fsm = NavFsm(
        robot, nav_cfg(mission__mode="target", mission__search="scan"), projector=projector
    )
    cmd = fsm.update(alive())
    assert cmd.v == 0.0 and cmd.omega != 0.0


def test_target_mode_servos_once_a_track_is_confirmed(robot, projector):
    nav = nav_cfg(mission__mode="target")
    fsm = NavFsm(robot, nav, projector=projector)
    for i in range(6):
        fsm.tracker.update([(GroundPoint(1.0, 0.1), 0.9, "bolt")], i / 30.0, Pose2D())
    cmd = fsm.update(alive(now=0.3))
    assert fsm.state is State.SERVO
    assert cmd.v > 0 and cmd.omega > 0


def test_after_collecting_one_it_goes_back_to_watching(robot, projector):
    # "Throw again, robot follows" is the demo, so arriving is not the end of
    # the mission.
    from fodnav.servo import ServoState

    fsm = NavFsm(robot, nav_cfg(mission__mode="target"), projector=projector)
    fsm.servo.state = ServoState.ARRIVED
    cmd = fsm.update(alive(now=5.0))
    assert fsm.state is State.SEARCH
    assert fsm.n_targets_collected == 1
    assert fsm.servo.state is ServoState.IDLE


def test_the_magnet_runs_while_the_mission_is_active(robot, projector):
    fsm = NavFsm(robot, nav_cfg(mission__mode="target"), projector=projector)
    fsm.update(alive())
    assert fsm.magnet_should_be_on
    fsm.update(alive(now=1.0, vision_age_s=99.0))
    assert not fsm.magnet_should_be_on, "a stopped robot should not spin the drum"


def test_the_magnet_can_be_switched_off_entirely(robot, projector):
    fsm = NavFsm(robot, nav_cfg(mission__mode="target", mission__magnet_on=False),
                 projector=projector)
    fsm.update(alive())
    assert not fsm.magnet_should_be_on


# -- perception ingestion -------------------------------------------------


def test_unknown_class_detections_never_become_tracks(robot, projector):
    from fodnav.link.detections import parse_message
    import json

    fsm = NavFsm(robot, nav_cfg(mission__mode="target"), projector=projector)
    msg = {
        "schema": 1, "t_capture": 1.0, "t_publish": 1.0, "frame_id": 1,
        "frame_size": [2304, 1296],
        "dets": [{"cls": "unknown", "conf": 0.99, "bbox": [1100, 900, 60, 40]}],
    }
    for i in range(10):
        fsm.update(alive(now=i / 30.0, frames=[parse_message(json.dumps(msg))]))
    assert fsm.tracker.tracks == []
    assert fsm.state is State.SEARCH


def test_a_frame_size_mismatch_is_allowed_to_stop_the_run(robot, projector):
    # If the resolution does not match the calibration, every projection is
    # silently wrong. Raising into the control loop stops the robot, which is
    # the correct outcome -- far better than driving on bad numbers.
    from fodnav.ground import GroundCalibrationError
    from fodnav.link.detections import parse_message
    import json

    fsm = NavFsm(robot, nav_cfg(mission__mode="target"), projector=projector)
    msg = {
        "schema": 1, "t_capture": 1.0, "t_publish": 1.0, "frame_id": 1,
        "frame_size": [640, 480],
        "dets": [{"cls": "bolt", "conf": 0.9, "bbox": [300, 400, 20, 20]}],
    }
    with pytest.raises(GroundCalibrationError):
        fsm.update(alive(frames=[parse_message(json.dumps(msg))]))
