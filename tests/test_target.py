"""Projecting his tracked pixels onto the floor, and ageing them.

Association and confidence hysteresis are not tested here because they are not
done here any more -- the vision library does both, and re-implementing them
would put two hysteresis loops in series. What is left is projection, staleness
and the odometry ageing, which is all that is ours.
"""

from __future__ import annotations

import math

import pytest

from fodnav.config import load_nav_config, load_robot_config
from fodnav.frames import Pose2D
from fodnav.ground import GroundProjector
from fodnav.link.vision import CAUTION, CONFIRM, IGNORE, PICK, parse_detail
from fodnav.sim.camera import SimCamera
from fodnav.target import TargetParams, TargetSet


@pytest.fixture(scope="module")
def robot():
    return load_robot_config("config/sim_robot.yaml")


@pytest.fixture(scope="module")
def projector(robot):
    cam = SimCamera(robot)
    return GroundProjector(cam.calibration(), max_valid_range_m=robot.get("camera.fov_far_limit_m"))


@pytest.fixture(scope="module")
def cam(robot):
    return SimCamera(robot)


def frame_with(cam, objects, ids=None, states=None):
    """A detail() frame holding the given (x, y) objects, via the real optics."""
    from fodnav.sim.camera import SimObject

    tracks = []
    for i, (x, y) in enumerate(objects):
        bbox = cam.bbox_for(SimObject(x=x, y=y))
        assert bbox is not None, f"object at {(x, y)} is not in frame"
        bx, by, w, h = bbox
        box = [int(bx), int(by), int(bx + w), int(by + h)]
        tracks.append({
            "id": (ids or [901 + i for i in range(len(objects))])[i],
            "state": (states or [CONFIRM] * len(objects))[i],
            "action": PICK, "cls": "bolt", "conf": 0.9, "raw": 0.9,
            "hits": 5, "misses": 0, "box": box,
            "centroid": [(box[0] + box[2]) / 2, (box[1] + box[3]) / 2], "in_zone": True,
        })
    return parse_detail({
        "frame_id": 1, "age": 0.0, "blocked": bool(tracks),
        "camera": {"frame_size": [cam.width_px, cam.height_px]},
        "tracks": tracks, "error": None,
    })


def test_his_confirmed_targets_land_on_the_floor(cam, projector):
    ts = TargetSet()
    live = ts.update(frame_with(cam, [(0.8, 0.15)]), projector, Pose2D(), 0.0)
    assert len(live) == 1
    tr = live[0]
    # The bottom edge is the object's near edge, so it reads half an object
    # length short -- a constant offset, unlike the centroid's growing one.
    assert tr.x == pytest.approx(0.8 - 0.03, abs=6e-3)
    assert tr.y == pytest.approx(0.15, abs=1e-2)


def test_his_track_id_is_carried_through_unchanged(cam, projector):
    # We do not re-key or re-associate. "The same screw across an approach" is
    # his answer and we keep his number so a log can be read against his.
    ts = TargetSet()
    ts.update(frame_with(cam, [(0.9, 0.0)], ids=[1477]), projector, Pose2D(), 0.0)
    assert list(ts.tracks) == [1477]
    ts.update(frame_with(cam, [(0.85, 0.0)], ids=[1477]), projector, Pose2D(), 0.05)
    assert list(ts.tracks) == [1477], "same id must update, not duplicate"


def test_only_confirmed_pickable_targets_are_kept(cam, projector):
    ts = TargetSet()
    live = ts.update(
        frame_with(cam, [(0.8, 0.0), (1.0, 0.2)], states=[CONFIRM, CAUTION]),
        projector, Pose2D(), 0.0,
    )
    assert len(live) == 1


def test_two_objects_stay_two_tracks(cam, projector):
    ts = TargetSet()
    live = ts.update(frame_with(cam, [(0.8, -0.15), (1.1, 0.2)]), projector, Pose2D(), 0.0)
    assert len(live) == 2


def test_the_nearest_track_is_the_one_chosen(cam, projector):
    ts = TargetSet()
    ts.update(frame_with(cam, [(1.2, 0.0), (0.7, 0.1)]), projector, Pose2D(), 0.0)
    best = ts.best(Pose2D())
    assert best.range_m < 0.8


def test_a_track_beyond_the_range_limit_is_not_chosen(cam, projector):
    ts = TargetSet()
    ts.update(frame_with(cam, [(1.2, 0.0)]), projector, Pose2D(), 0.0)
    assert ts.best(Pose2D(), max_range_m=0.5) is None
    assert ts.best(Pose2D(), max_range_m=2.0) is not None


def test_a_track_goes_stale_if_frames_stop(cam, projector):
    ts = TargetSet(TargetParams(max_age_s=0.2))
    ts.update(frame_with(cam, [(0.9, 0.0)]), projector, Pose2D(), 0.0)
    assert ts.tracks
    ts.update(parse_detail({"frame_id": 2, "age": 0.0, "blocked": False,
                            "camera": {"frame_size": [cam.width_px, cam.height_px]},
                            "tracks": [], "error": None}),
              projector, Pose2D(), 0.5)
    assert ts.tracks == {}


def test_a_stale_measurement_is_aged_forward_on_odometry(cam, projector):
    # The loop runs at 50 Hz on his 30 Hz stream, so most ticks use a
    # measurement up to 33 ms old -- about a centimetre, enough to move where
    # the blind leg latches.
    ts = TargetSet()
    ts.update(frame_with(cam, [(1.0, 0.0)]), projector, Pose2D(0, 0, 0), 0.0)
    tr = next(iter(ts.tracks.values()))
    measured = tr.x
    advanced = tr.predict_base(Pose2D(0.10, 0.0, 0.0))
    assert advanced.x == pytest.approx(measured - 0.10, abs=1e-9)
    assert tr.x == measured, "the stored measurement is not mutated"


def test_ageing_handles_rotation(cam, projector):
    ts = TargetSet()
    ts.update(frame_with(cam, [(1.0, 0.0)]), projector, Pose2D(0, 0, 0), 0.0)
    tr = next(iter(ts.tracks.values()))
    p = tr.predict_base(Pose2D(0.0, 0.0, math.pi / 2))
    # Turning the robot 90 deg left swings a point at (x, y) round to (y, -x).
    assert (p.x, p.y) == pytest.approx((tr.y, -tr.x), abs=1e-9)


def test_a_frame_at_the_wrong_resolution_is_refused(cam, projector):
    from fodnav.ground import GroundCalibrationError

    ts = TargetSet()
    bad = parse_detail({"frame_id": 1, "age": 0.0, "blocked": False,
                        "camera": {"frame_size": [640, 480]}, "tracks": [], "error": None})
    with pytest.raises(GroundCalibrationError):
        ts.update(bad, projector, Pose2D(), 0.0)


def test_the_shipped_backstop_is_configured():
    nav = load_nav_config("config/nav.yaml")
    assert TargetParams.from_config(nav).max_age_s > 0
