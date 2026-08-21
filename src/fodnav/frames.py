"""Frame conventions, rigid transforms, and the one angle-wrap function.

Pure module: imports nothing from ``fodnav``, touches no I/O, holds no state.

Conventions are fixed by CLAUDE.md section 3 and are not negotiable per-module.
Right-handed, z up, yaw positive counter-clockwise from +x (REP-103).

Frames
------
``world``  Fixed. Origin at the arena's designated corner, +x along the long
           axis, +y left. All planned waypoints live here.
``odom``   The robot's integrated pose. Continuous and smooth but drifting.
           Starts coincident with ``world``.
``base``   Robot body. Origin at the midpoint of the drive-wheel axle projected
           onto the floor. +x forward, +y left, +z up. Not the chassis centre,
           not the camera.
``cam``    Camera optical frame, standard optical convention: +z out of the
           lens, +x right, +y down. Fixed mount transform from ``base``.

Units are metres, radians and seconds. Everywhere. Degrees exist in
``config/robot.yaml`` (an inclinometer reads degrees) and at the display
boundary; the config loader is where they stop.

Naming: a transform is written ``parent_T_child`` and read as "the pose of
``child`` expressed in ``parent``". Composition is left-to-right:
``world_T_cam = world_T_base @ base_T_cam``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

__all__ = [
    "wrap_angle",
    "angle_diff",
    "Pose2D",
    "CameraMount",
    "rot_x",
    "rot_y",
    "rot_z",
    "R_BASE_FROM_OPTICAL",
]

TAU = 2.0 * math.pi


def wrap_angle(a):
    """Wrap an angle into ``(-pi, pi]``.

    This is *the* wrap function. There is not a second one anywhere in this
    repo, and an angle written into state without passing through here is a
    bug waiting for the robot to cross the +/-pi seam at the worst moment.

    Scalars in, float out; numpy arrays in, array out.

    Note the half-open interval: exactly ``-pi`` wraps to ``+pi``. The choice
    matters only at the seam, but it has to be made once and stated.
    """
    return math.pi - (math.pi - a) % TAU


def angle_diff(a, b):
    """Signed smallest rotation taking ``b`` to ``a``, in ``(-pi, pi]``."""
    return wrap_angle(a - b)


@dataclass(frozen=True, slots=True)
class Pose2D:
    """A planar pose, and equivalently a planar rigid transform.

    ``theta`` is wrapped on construction, so a ``Pose2D`` in hand is always
    wrapped. That is the enforcement mechanism behind "never write an atan2
    result into state without wrapping" -- write it into a Pose2D instead.
    """

    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0

    def __post_init__(self) -> None:
        object.__setattr__(self, "x", float(self.x))
        object.__setattr__(self, "y", float(self.y))
        object.__setattr__(self, "theta", float(wrap_angle(self.theta)))

    # -- composition ----------------------------------------------------

    def compose(self, other: "Pose2D") -> "Pose2D":
        """``self`` is ``a_T_b`` and ``other`` is ``b_T_c``; return ``a_T_c``."""
        c, s = math.cos(self.theta), math.sin(self.theta)
        return Pose2D(
            self.x + c * other.x - s * other.y,
            self.y + s * other.x + c * other.y,
            self.theta + other.theta,
        )

    __matmul__ = compose

    def inverse(self) -> "Pose2D":
        """``a_T_b`` -> ``b_T_a``."""
        c, s = math.cos(self.theta), math.sin(self.theta)
        return Pose2D(-(c * self.x + s * self.y), -(-s * self.x + c * self.y), -self.theta)

    def relative_to(self, other: "Pose2D") -> "Pose2D":
        """Express ``self`` in the frame of ``other``.

        Both are poses in a common parent. The result is what ``self`` looks
        like from ``other`` -- e.g. a goal in ``world`` and the robot in
        ``world`` gives the goal in ``base``, which is exactly what every
        controller in this repo wants.
        """
        return other.inverse() @ self

    # -- points ---------------------------------------------------------

    def transform_point(self, x: float, y: float) -> tuple[float, float]:
        """Map a point from the child frame into the parent frame."""
        c, s = math.cos(self.theta), math.sin(self.theta)
        return (self.x + c * x - s * y, self.y + s * x + c * y)

    def inverse_transform_point(self, x: float, y: float) -> tuple[float, float]:
        """Map a point from the parent frame into the child frame."""
        c, s = math.cos(self.theta), math.sin(self.theta)
        dx, dy = x - self.x, y - self.y
        return (c * dx + s * dy, -s * dx + c * dy)

    # -- interop --------------------------------------------------------

    def as_matrix(self) -> np.ndarray:
        c, s = math.cos(self.theta), math.sin(self.theta)
        return np.array([[c, -s, self.x], [s, c, self.y], [0.0, 0.0, 1.0]])

    @staticmethod
    def from_matrix(m: np.ndarray) -> "Pose2D":
        return Pose2D(float(m[0, 2]), float(m[1, 2]), math.atan2(float(m[1, 0]), float(m[0, 0])))

    @property
    def xy(self) -> tuple[float, float]:
        return (self.x, self.y)

    def distance_to(self, other: "Pose2D") -> float:
        return math.hypot(other.x - self.x, other.y - self.y)

    def __repr__(self) -> str:  # metres and radians, but degrees help humans
        return (
            f"Pose2D(x={self.x:.3f}, y={self.y:.3f}, "
            f"theta={self.theta:.3f} rad [{math.degrees(self.theta):.1f} deg])"
        )


# -- 3D, for the camera mount only ---------------------------------------


def rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


#: Optical-to-base rotation for a camera looking straight down the +x axis of
#: ``base`` with zero tilt and zero roll. Columns are the camera's axes
#: expressed in ``base``: optical +x (right) is base -y, optical +y (down) is
#: base -z, optical +z (forward, out of the lens) is base +x.
R_BASE_FROM_OPTICAL = np.array(
    [
        [0.0, 0.0, 1.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
    ]
)


@dataclass(frozen=True)
class CameraMount:
    """The fixed ``base_T_cam`` transform, from measured mount geometry.

    All arguments are SI: metres and radians. ``config/robot.yaml`` stores the
    tilt and roll in degrees because that is what an inclinometer reads; the
    config loader converts, and nothing downstream sees a degree.

    ``tilt_rad`` is *down*-tilt, positive, measured from horizontal: it takes
    the optical axis from base +x toward base -z. ``roll_rad`` is rotation
    about the optical axis and should be zero; a twisted mount shears every
    projection, which is why HARDWARE.md section 3 asks for it to be measured
    anyway.

    Composition order: roll is applied in the optical frame, then the mount
    down-tilt. For the small roll of a mount that is meant to be level the two
    orders differ negligibly, but the order is fixed here so that the ground
    calibration and the pinhole fallback cannot disagree about it.

    This transform is valid only for the geometry it was measured at. If the
    mount moves, every number here and the whole ground calibration are void.
    """

    height_m: float
    tilt_rad: float
    offset_x_m: float = 0.0
    offset_y_m: float = 0.0
    roll_rad: float = 0.0

    R: np.ndarray = field(init=False, repr=False)
    t: np.ndarray = field(init=False, repr=False)

    def __post_init__(self) -> None:
        R = rot_y(self.tilt_rad) @ R_BASE_FROM_OPTICAL @ rot_z(self.roll_rad)
        object.__setattr__(self, "R", R)
        object.__setattr__(
            self, "t", np.array([self.offset_x_m, self.offset_y_m, self.height_m])
        )

    def base_from_cam_point(self, p_cam: np.ndarray) -> np.ndarray:
        """Map a point (or Nx3 stack of points) from ``cam`` into ``base``."""
        p = np.asarray(p_cam, dtype=float)
        return p @ self.R.T + self.t

    def base_from_cam_direction(self, d_cam: np.ndarray) -> np.ndarray:
        """Rotate a direction (no translation) from ``cam`` into ``base``."""
        d = np.asarray(d_cam, dtype=float)
        return d @ self.R.T

    @property
    def optical_axis_base(self) -> np.ndarray:
        """Where the camera is looking, as a unit vector in ``base``."""
        return self.R[:, 2]
