"""Ground projection: the numbers, and the three ways it must refuse to answer."""

from __future__ import annotations

import json
import math

import numpy as np
import pytest

from fodnav.config import load_robot_config
from fodnav.frames import CameraMount
from fodnav.ground import (
    GroundCalibration,
    GroundCalibrationError,
    GroundProjector,
    fit_homography,
    homography_from_pinhole,
    intrinsics_from_fov,
    load_ground_calibration,
    mount_from_config,
)
from fodnav.link.detections import Detection
from fodnav.sim.camera import SimCamera

SIM_YAML = "config/sim_robot.yaml"


@pytest.fixture(scope="module")
def robot():
    return load_robot_config(SIM_YAML)


@pytest.fixture(scope="module")
def cam(robot):
    return SimCamera(robot)


@pytest.fixture(scope="module")
def projector(cam, robot):
    return GroundProjector(cam.calibration(), max_valid_range_m=robot.get("camera.fov_far_limit_m"))


# -- the fit --------------------------------------------------------------


def test_a_fitted_homography_recovers_the_points_it_was_fitted_to(cam):
    pixel, floor = cam.marker_grid()
    H, res = fit_homography(pixel, floor)
    assert res["n_points"] >= 6, "CLAUDE.md §7 asks for at least six markers"
    assert res["rms_m"] < 1e-4
    for (u, v), (x, y) in zip(pixel, floor):
        q = H @ np.array([u, v, 1.0])
        assert (q[0] / q[2], q[1] / q[2]) == pytest.approx((x, y), abs=1e-4)


def test_the_fit_reports_residuals_in_metres(cam):
    # The residual is the number that says whether an afternoon with the tape
    # measure produced a calibration or produced a warning.
    pixel, floor = cam.marker_grid()
    noisy = [[u + 3.0, v - 3.0] for u, v in pixel]
    _, res = fit_homography(noisy, floor)
    assert res["rms_m"] > 0.0


def test_four_points_is_the_mathematical_floor(cam):
    pixel, floor = cam.marker_grid()
    with pytest.raises(ValueError, match="at least 4"):
        fit_homography(pixel[:3], floor[:3])


def test_mismatched_point_counts_are_rejected(cam):
    pixel, floor = cam.marker_grid()
    with pytest.raises(ValueError, match="pixel points but"):
        fit_homography(pixel, floor[:-1])


def test_a_fitted_homography_matches_the_analytic_one(cam):
    # A pinhole's view of a plane IS a homography, so fitting to noiseless
    # synthetic markers must recover the analytic map. If this drifts, the
    # simulated pipeline has stopped being faithful and every sim result is
    # about the fake rather than about nav.
    fitted = cam.calibration(fit=True).H_pixel_to_floor
    analytic = cam.H_pixel_to_floor
    fitted = fitted / fitted[2, 2]
    analytic = analytic / analytic[2, 2]
    assert np.allclose(fitted, analytic, rtol=1e-6, atol=1e-9)


# -- projection -----------------------------------------------------------


@pytest.mark.parametrize("x,y", [(0.35, 0.0), (0.6, 0.2), (1.0, -0.3), (1.4, 0.45)])
def test_pixels_and_metres_round_trip(projector, x, y):
    u, v = projector.pixel_from_floor(x, y)
    g = projector.project_pixel(u, v)
    assert g is not None
    assert g.xy == pytest.approx((x, y), abs=1e-6)


def test_range_and_bearing_are_the_polar_form_of_the_same_point(projector):
    u, v = projector.pixel_from_floor(0.8, 0.4)
    g = projector.project_pixel(u, v)
    assert g.range_m == pytest.approx(math.hypot(0.8, 0.4))
    assert g.bearing_rad == pytest.approx(math.atan2(0.4, 0.8))
    assert g.bearing_rad > 0, "a target to the left must have a positive bearing"


def test_the_optical_axis_lands_where_trigonometry_says(robot, projector):
    h = robot.get("camera.height_m")
    t = robot.get("camera.tilt_rad")
    x0 = robot.get("camera.offset_x_m")
    g = projector.project_pixel(
        robot.get("camera.capture_width_px") / 2.0, robot.get("camera.capture_height_px") / 2.0
    )
    assert g.x == pytest.approx(x0 + h / math.tan(t), abs=1e-3)
    assert g.y == pytest.approx(0.0, abs=1e-6)


# -- the three refusals ---------------------------------------------------


def test_the_horizon_is_rejected_rather_than_extrapolated(cam, projector):
    # A ray that does not point downward has no floor intersection. Returning a
    # huge or negative range instead of nothing is how a robot decides to drive
    # to a light fitting.
    row = cam.horizon_row()
    assert row is not None, "the fictional camera should see its own horizon"
    assert projector.project_pixel(cam.width_px / 2.0, row) is None
    assert projector.project_pixel(cam.width_px / 2.0, row - 5.0) is None
    assert projector.project_pixel(cam.width_px / 2.0, 0.0) is None
    assert projector.stats()["rejected_horizon"] >= 3


def test_just_below_the_horizon_still_projects(cam, projector):
    g = projector.project_pixel(cam.width_px / 2.0, cam.horizon_row() + 40.0)
    # It is a real floor point, just a distant one -- possibly beyond the range
    # gate, which is the *other* refusal.
    assert g is None or g.x > 1.0


def test_beyond_the_calibrated_patch_is_rejected(projector, robot):
    far = robot.get("camera.fov_far_limit_m")
    u, v = projector.pixel_from_floor(far * 2.0, 0.0)
    assert projector.project_pixel(u, v) is None
    u, v = projector.pixel_from_floor(far * 0.9, 0.0)
    assert projector.project_pixel(u, v) is not None


def test_a_frame_size_mismatch_is_an_error_not_a_warning(projector):
    # If the resolution does not match, the homography is invalid and every
    # projection is silently wrong (CLAUDE.md §8).
    det = Detection(cls="bolt", conf=0.9, x=1100.0, y=800.0, w=60.0, h=40.0)
    projector.project_detection(det, frame_size=projector.frame_size)  # fine
    with pytest.raises(GroundCalibrationError, match="frame_size|silently wrong"):
        projector.project_detection(det, frame_size=(640, 480))


def test_the_letterboxed_network_input_is_caught_by_that_check(projector):
    with pytest.raises(GroundCalibrationError):
        projector.check_frame_size((480, 480))


# -- bottom-centre, and why ----------------------------------------------


def _det_for(cam, obj):
    from fodnav.link.detections import Detection as D

    bbox = cam.bbox_for(obj)
    assert bbox is not None, f"object at x={obj.x} is not in frame"
    return D(cls="bolt", conf=0.9, x=bbox[0], y=bbox[1], w=bbox[2], h=bbox[3])


def test_the_bottom_edge_lands_on_the_near_edge_of_the_object_at_any_range(cam, projector):
    # The bottom of the box is where the object's nearest part touches the
    # floor, so the projection is offset by half the object's length -- and
    # that offset is CONSTANT with range. A constant offset is something you
    # can reason about, and it is a centimetre or two on a nail.
    from fodnav.sim.camera import SimObject

    for x in (0.4, 0.7, 1.1, 1.5):
        obj = SimObject(x=x, y=0.1, length_m=0.06)
        g = projector.project_pixel(*_det_for(cam, obj).ground_px)
        assert g.x == pytest.approx(x - obj.length_m / 2.0, abs=2e-3)
        assert g.y == pytest.approx(0.1, abs=1e-2)


def test_the_bottom_edge_does_not_care_how_tall_the_object_is(cam, projector):
    from fodnav.sim.camera import SimObject

    flat = projector.project_pixel(*_det_for(cam, SimObject(x=0.7, height_m=0.005)).ground_px)
    tall = projector.project_pixel(*_det_for(cam, SimObject(x=0.7, height_m=0.050)).ground_px)
    assert tall.x == pytest.approx(flat.x, abs=2e-3)


def test_the_centroid_over_ranges_and_gets_worse_with_height_and_distance(cam, projector):
    # This is the whole reason CLAUDE.md §3 names the bottom edge. Unlike the
    # half-length offset above, this error is not constant: it grows with both
    # object height and range, which is exactly what a bad calibration looks
    # like. Someone would spend two days re-taping markers to the floor.
    from fodnav.sim.camera import SimObject

    errors = []
    for x in (0.4, 0.7, 1.1):
        det = _det_for(cam, SimObject(x=x, height_m=0.05))
        centre = projector.project_pixel(*det.centre_px)
        bottom = projector.project_pixel(*det.ground_px)
        errors.append(centre.x - x)
        assert centre.x > bottom.x
    assert errors == sorted(errors), "the centroid's range error must grow with distance"
    assert errors[-1] > 0.10, "and it gets large: over 10 cm at ~1 m on a 5 cm object"

    flat = projector.project_pixel(*_det_for(cam, SimObject(x=0.7, height_m=0.005)).centre_px)
    tall = projector.project_pixel(*_det_for(cam, SimObject(x=0.7, height_m=0.050)).centre_px)
    assert tall.x > flat.x + 0.05, "and with object height"


# -- the file, and the assertion that guards it ---------------------------


def test_calibration_round_trips_through_json(tmp_path, cam):
    calib = cam.calibration()
    p = tmp_path / "ground_homography.json"
    calib.save(p)
    back = load_ground_calibration(p)
    assert np.allclose(back.H_pixel_to_floor, calib.H_pixel_to_floor)
    assert back.frame_size == calib.frame_size
    assert back.residuals["n_points"] == calib.residuals["n_points"]


def test_loading_asserts_the_calibration_matches_the_robot(tmp_path, cam, robot):
    p = tmp_path / "ground_homography.json"
    cam.calibration().save(p)
    load_ground_calibration(p, robot)  # matches, so it loads


@pytest.mark.parametrize(
    "field, value",
    [("height_m", 0.30), ("tilt_deg", 25.0), ("offset_x_m", 0.2), ("roll_deg", 2.0)],
)
def test_a_moved_mount_voids_the_calibration(tmp_path, cam, robot, field, value):
    # Any change to the mount invalidates it. Silently using the old one gives
    # numbers that are wrong by a fixed amount, which reads as a controller
    # problem for as long as anyone will keep tuning gains.
    d = cam.calibration().to_json()
    d["camera"][field] = value
    p = tmp_path / "ground_homography.json"
    p.write_text(json.dumps(d))
    with pytest.raises(GroundCalibrationError, match=field):
        load_ground_calibration(p, robot)


def test_a_recalibration_at_another_resolution_is_caught(tmp_path, cam, robot):
    d = cam.calibration().to_json()
    d["frame_size"] = [1920, 1080]
    p = tmp_path / "ground_homography.json"
    p.write_text(json.dumps(d))
    with pytest.raises(GroundCalibrationError, match="resolution"):
        load_ground_calibration(p, robot)


def test_a_missing_calibration_says_what_to_run(tmp_path):
    with pytest.raises(GroundCalibrationError, match="fodnav-calib-ground"):
        load_ground_calibration(tmp_path / "nope.json")


def test_an_old_schema_is_rejected(tmp_path, cam):
    d = cam.calibration().to_json()
    d["schema"] = 0
    p = tmp_path / "g.json"
    p.write_text(json.dumps(d))
    with pytest.raises(GroundCalibrationError, match="schema"):
        load_ground_calibration(p)


# -- the mount transform underneath it -----------------------------------


def test_the_analytic_homography_agrees_with_the_mount_transform(robot):
    mount = mount_from_config(robot)
    K = intrinsics_from_fov(2304, 1296, math.radians(102.0))
    H = homography_from_pinhole(mount, K)
    # Project a known floor point forward through K and the mount, and back
    # through H. Two independent paths through the same geometry.
    p_base = np.array([0.8, 0.15, 0.0])
    p_cam = mount.R.T @ (p_base - mount.t)
    q = K @ p_cam
    u, v = q[0] / q[2], q[1] / q[2]
    r = H @ np.array([u, v, 1.0])
    assert (r[0] / r[2], r[1] / r[2]) == pytest.approx((0.8, 0.15), abs=1e-9)


def test_an_unmeasured_mount_refuses_to_build(tmp_path):
    from fodnav.config import MissingValueError

    real = load_robot_config("config/robot.yaml")
    with pytest.raises(MissingValueError, match="camera"):
        mount_from_config(real)
