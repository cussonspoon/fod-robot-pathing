"""Frame conventions. If these are wrong, every other test is testing nothing."""

from __future__ import annotations

import math

import numpy as np
import pytest

from fodnav.frames import (
    R_BASE_FROM_OPTICAL,
    CameraMount,
    Pose2D,
    angle_diff,
    wrap_angle,
)


# -- wrap_angle -----------------------------------------------------------


@pytest.mark.parametrize("a", np.linspace(-40.0, 40.0, 401))
def test_wrap_lands_in_the_half_open_interval(a):
    w = wrap_angle(a)
    assert -math.pi < w <= math.pi


@pytest.mark.parametrize("a", np.linspace(-40.0, 40.0, 401))
def test_wrap_preserves_the_angle_modulo_tau(a):
    w = wrap_angle(a)
    assert math.isclose(math.cos(w), math.cos(a), abs_tol=1e-12)
    assert math.isclose(math.sin(w), math.sin(a), abs_tol=1e-12)


def test_wrap_is_idempotent():
    for a in np.linspace(-20.0, 20.0, 201):
        assert wrap_angle(wrap_angle(a)) == pytest.approx(wrap_angle(a), abs=1e-15)


def test_the_seam_is_closed_at_plus_pi():
    # The interval is (-pi, pi], so -pi comes back as +pi. This is the one
    # place the convention is observable; pin it down.
    assert wrap_angle(math.pi) == pytest.approx(math.pi)
    assert wrap_angle(-math.pi) == pytest.approx(math.pi)


def test_wrap_handles_arrays():
    a = np.array([0.0, math.pi, -math.pi, 7.0, -7.0])
    w = wrap_angle(a)
    assert w.shape == a.shape
    assert np.all(w > -math.pi) and np.all(w <= math.pi)


def test_angle_diff_is_the_short_way_round():
    assert angle_diff(0.1, -0.1) == pytest.approx(0.2)
    # Across the seam: 179 deg to -179 deg is 2 deg, not 358.
    assert angle_diff(math.radians(-179), math.radians(179)) == pytest.approx(
        math.radians(2), abs=1e-12
    )


# -- Pose2D ---------------------------------------------------------------


POSES = [
    Pose2D(0, 0, 0),
    Pose2D(1.5, -2.25, 0.7),
    Pose2D(-3.0, 0.5, -2.9),
    Pose2D(0.01, 0.02, math.pi),
]


def test_theta_is_wrapped_on_construction():
    assert Pose2D(0, 0, 3 * math.pi).theta == pytest.approx(math.pi)
    assert Pose2D(0, 0, 7.0).theta == pytest.approx(wrap_angle(7.0))


@pytest.mark.parametrize("p", POSES)
def test_inverse_round_trips(p):
    i = p @ p.inverse()
    assert i.x == pytest.approx(0.0, abs=1e-12)
    assert i.y == pytest.approx(0.0, abs=1e-12)
    assert i.theta == pytest.approx(0.0, abs=1e-12)


@pytest.mark.parametrize("p", POSES)
def test_point_transform_round_trips(p):
    for x, y in [(0.0, 0.0), (1.0, 0.0), (-2.5, 3.5)]:
        assert p.inverse_transform_point(*p.transform_point(x, y)) == pytest.approx((x, y), abs=1e-12)


def test_composition_matches_matrix_multiplication():
    a, b = POSES[1], POSES[2]
    assert np.allclose((a @ b).as_matrix(), a.as_matrix() @ b.as_matrix())


def test_composition_is_associative():
    a, b, c = POSES[1], POSES[2], POSES[3]
    left, right = (a @ b) @ c, a @ (b @ c)
    assert (left.x, left.y) == pytest.approx((right.x, right.y), abs=1e-12)
    assert left.theta == pytest.approx(right.theta, abs=1e-12)


@pytest.mark.parametrize("p", POSES)
def test_matrix_round_trips(p):
    q = Pose2D.from_matrix(p.as_matrix())
    assert (q.x, q.y, q.theta) == pytest.approx((p.x, p.y, p.theta), abs=1e-12)


def test_relative_to_puts_a_world_goal_in_the_base_frame():
    # Robot at (1, 1) facing +y (90 deg). A goal at (1, 3) in world is 2 m
    # straight ahead of it: (2, 0) in base.
    robot = Pose2D(1.0, 1.0, math.pi / 2)
    goal = Pose2D(1.0, 3.0, 0.0)
    g = goal.relative_to(robot)
    assert (g.x, g.y) == pytest.approx((2.0, 0.0), abs=1e-12)
    # ...and the goal's heading is 90 deg to its right.
    assert g.theta == pytest.approx(-math.pi / 2)


def test_a_pose_composed_with_a_pure_rotation_turns_in_place():
    p = Pose2D(2.0, 0.0, 0.0) @ Pose2D(0.0, 0.0, math.pi / 2)
    assert (p.x, p.y) == pytest.approx((2.0, 0.0))
    assert p.theta == pytest.approx(math.pi / 2)


# -- camera mount ---------------------------------------------------------


def test_optical_convention_at_zero_tilt():
    m = CameraMount(height_m=0.25, tilt_rad=0.0)
    # +z out of the lens is base forward, +x right is base -y, +y down is -z.
    assert np.allclose(m.base_from_cam_direction(np.array([0.0, 0.0, 1.0])), [1, 0, 0])
    assert np.allclose(m.base_from_cam_direction(np.array([1.0, 0.0, 0.0])), [0, -1, 0])
    assert np.allclose(m.base_from_cam_direction(np.array([0.0, 1.0, 0.0])), [0, 0, -1])
    assert np.allclose(m.R, R_BASE_FROM_OPTICAL)


@pytest.mark.parametrize("tilt_deg", [5.0, 10.0, 18.0, 25.0, 45.0])
def test_down_tilt_points_the_optical_axis_forward_and_down(tilt_deg):
    t = math.radians(tilt_deg)
    m = CameraMount(height_m=0.22, tilt_rad=t)
    axis = m.optical_axis_base
    assert axis == pytest.approx([math.cos(t), 0.0, -math.sin(t)], abs=1e-12)
    # Down-tilt must not roll the horizon: the camera's right stays horizontal.
    assert m.base_from_cam_direction(np.array([1.0, 0.0, 0.0])) == pytest.approx([0, -1, 0], abs=1e-12)


def test_rotation_is_orthonormal_with_roll_and_tilt():
    m = CameraMount(height_m=0.2, tilt_rad=0.3, roll_rad=0.05)
    assert np.allclose(m.R @ m.R.T, np.eye(3), atol=1e-12)
    assert np.linalg.det(m.R) == pytest.approx(1.0)


def test_the_optical_axis_hits_the_floor_where_trigonometry_says_it_does():
    # A camera at height h tilted down by t, mounted x0 forward of the axle,
    # looks at the floor at x0 + h/tan(t). This is the sanity check every
    # ground-projection bug fails.
    h, t, x0 = 0.22, math.radians(18.0), 0.10
    m = CameraMount(height_m=h, tilt_rad=t, offset_x_m=x0)
    d = m.optical_axis_base
    s = m.t[2] / -d[2]  # distance along the ray to z = 0
    hit = m.t + s * d
    assert hit[2] == pytest.approx(0.0, abs=1e-12)
    assert hit[0] == pytest.approx(x0 + h / math.tan(t), abs=1e-9)
    assert hit[1] == pytest.approx(0.0, abs=1e-12)


def test_camera_offsets_translate_but_do_not_rotate():
    m = CameraMount(height_m=0.2, tilt_rad=0.2, offset_x_m=0.1, offset_y_m=-0.03)
    assert m.base_from_cam_point(np.zeros(3)) == pytest.approx([0.1, -0.03, 0.2])
