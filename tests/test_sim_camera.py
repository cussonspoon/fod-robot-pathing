"""The fictional camera. Its job is to be internally consistent, not real."""

from __future__ import annotations

import math

import pytest

from fodnav.config import load_robot_config
from fodnav.sim.camera import SimCamera, SimObject


@pytest.fixture(scope="module")
def robot():
    return load_robot_config("config/sim_robot.yaml")


@pytest.fixture(scope="module")
def cam(robot):
    return SimCamera(robot)


def test_the_declared_field_of_view_matches_the_camera_it_publishes_through(cam, robot):
    # sim_robot.yaml's camera block is derived from this model. If they drift,
    # the blind-leg handover fires at a distance the simulated camera does not
    # actually have, and the sim teaches us something false.
    assert cam.near_limit_m() == pytest.approx(robot.get("camera.fov_near_limit_m"), abs=1e-3)
    assert cam.width_at(0.30) == pytest.approx(
        robot.get("camera.fov_width_at_lookahead_m"), abs=1e-3
    )


def test_the_detection_swath_is_much_wider_than_the_drum(cam, robot):
    # Not an accident, and not a bug: CLAUDE.md §9 says the right swath width
    # depends on whether coverage means detection or collection, and these two
    # numbers are how far apart the two answers are.
    assert cam.width_at(0.30) > 2.0 * robot.get("drum.width_m")


def test_the_horizon_is_in_frame_so_the_guard_has_work_to_do(cam):
    row = cam.horizon_row()
    assert row is not None and 0 < row < cam.height_px


def test_the_bottom_of_the_box_is_the_objects_nearest_contact_with_the_floor(cam):
    # Not its centre: the box is the hull of the object, so its bottom edge is
    # the nearest floor-touching corner. Detectors behave this way and so does
    # this fake, which is why the projected ground point sits half an object
    # length short of the object's centre.
    obj = SimObject(x=0.8, y=0.1, length_m=0.06, height_m=0.03)
    x, y, w, h = cam.bbox_for(obj)
    near_edge = cam.project_point((obj.x - obj.length_m / 2.0, obj.y, 0.0))
    centre = cam.project_point((obj.x, obj.y, 0.0))
    assert (y + h) == pytest.approx(near_edge[1], abs=1.0)
    assert (y + h) > centre[1], "the bottom edge images below the object's centre"
    assert (x + w / 2.0) == pytest.approx(centre[0], abs=2.0)


def test_a_target_closer_than_the_near_limit_is_simply_not_seen(cam):
    # This is the terminal blind leg, in one assertion. The servo controller
    # cannot run to contact because the target stops existing first.
    assert cam.bbox_for(SimObject(x=cam.near_limit_m() - 0.05, y=0.0)) is None
    assert cam.bbox_for(SimObject(x=cam.near_limit_m() + 0.10, y=0.0)) is not None


def test_a_target_far_off_to_the_side_is_not_seen(cam):
    assert cam.bbox_for(SimObject(x=0.5, y=2.0)) is None


def test_boxes_shrink_with_distance(cam):
    areas = []
    for x in (0.5, 0.8, 1.2, 1.6):
        _, _, w, h = cam.bbox_for(SimObject(x=x, y=0.0))
        areas.append(w * h)
    assert areas == sorted(areas, reverse=True)


def test_the_marker_grid_is_not_collinear(cam):
    pixel, floor = cam.marker_grid()
    xs = {round(p[0], 3) for p in floor}
    ys = {round(p[1], 3) for p in floor}
    assert len(xs) >= 3 and len(ys) >= 3
