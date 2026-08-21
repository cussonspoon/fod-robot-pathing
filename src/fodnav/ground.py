"""Pixel -> floor-metre projection. Bottom-centre of the box, in ``base``.

This lives here and not in the CV repo because it depends on the physical
camera mount, and the CV repo must stay mount-agnostic so its benchmarks stay
portable (CLAUDE.md section 1). If you find yourself wanting to add extrinsics
over there, stop.

**Primary: a planar homography.** Tape at least six markers on the floor at
known ``base`` positions, capture one frame, find their pixel positions, solve
the 3x3 ``H`` that maps image pixels to floor metres. No camera intrinsics
needed and it absorbs lens distortion approximately over the calibrated region.
``fodnav-calib-ground`` runs this and writes ``config/ground_homography.json``.

**Fallback: pinhole plus extrinsics.** Calibrate ``K`` with a checkerboard,
back-project the pixel ray, intersect ``z = 0`` in ``base``. Use it only if the
homography proves unstable, or if you need to reason about points outside the
calibrated patch. It is *not built*: what is built here is the exact
plane-induced homography of a pinhole camera
(:func:`homography_from_pinhole`), which is the same map for points on the
floor and is what the simulated camera uses. A real pinhole fallback needs a
checkerboard ``K`` and a distortion model, and until the homography actually
proves unstable that work is not worth doing.

Three ways this returns nothing instead of a number, all deliberate:

* **The horizon.** A ray that does not point downward never meets the floor.
  Reject it; do not return a huge or negative range.
* **Out of range.** Extrapolating past the calibrated patch degrades fast, so
  a projected range beyond ``max_valid_range_m`` is rejected rather than
  trusted.
* **The wrong resolution.** A calibration is valid only for the frame size it
  was captured at, and the detection message carries ``frame_size`` precisely
  so this is checkable. A mismatch means every projection is silently wrong.

And the calibration as a whole is valid only for the mount geometry it was
taken at. Loading it asserts camera height, tilt and resolution against
``config/robot.yaml``. Any change to the mount voids it.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Sequence

import numpy as np

from .config import Config
from .frames import CameraMount

__all__ = [
    "CALIB_SCHEMA_VERSION",
    "GroundCalibration",
    "GroundCalibrationError",
    "GroundProjector",
    "fit_homography",
    "homography_from_pinhole",
    "intrinsics_from_fov",
    "load_ground_calibration",
    "projector_from_config",
]

CALIB_SCHEMA_VERSION = 1

#: How closely the mount geometry recorded in a calibration file must match
#: ``robot.yaml`` before the calibration is considered to describe this robot.
#: Tight on purpose: these are not values that drift, they are values someone
#: re-measured or a mount that moved.
_GEOMETRY_TOL = {
    "height_m": 1e-4,
    "tilt_deg": 1e-3,
    "offset_x_m": 1e-4,
    "offset_y_m": 1e-4,
    "roll_deg": 1e-3,
}


class GroundCalibrationError(Exception):
    """The calibration does not describe the robot that is asking."""


# ---------------------------------------------------------------------------
# fitting
# ---------------------------------------------------------------------------


def fit_homography(
    pixel_pts: Sequence[Sequence[float]], floor_pts: Sequence[Sequence[float]]
) -> tuple[np.ndarray, dict[str, float]]:
    """Solve the pixel -> floor homography and report how well it fits.

    Returns ``(H, residuals)`` where ``H`` maps homogeneous pixels to floor
    metres in ``base``, and ``residuals`` carries the RMS and worst
    reprojection error **in metres** -- which is the number that tells you
    whether the calibration is any good. A 3 cm RMS is not a calibration, it is
    a warning.

    At least four correspondences are needed mathematically; CLAUDE.md section 7
    asks for six or more, because four gives an exact fit with no residual to
    look at, which is indistinguishable from a perfect one.
    """
    import cv2  # the only OpenCV call in the runtime, hence the headless build

    px = np.asarray(pixel_pts, dtype=np.float64).reshape(-1, 2)
    fl = np.asarray(floor_pts, dtype=np.float64).reshape(-1, 2)
    if px.shape[0] != fl.shape[0]:
        raise ValueError(f"{px.shape[0]} pixel points but {fl.shape[0]} floor points")
    if px.shape[0] < 4:
        raise ValueError(
            f"a homography needs at least 4 correspondences, got {px.shape[0]}; "
            "six or more is what makes the residual meaningful"
        )
    H, mask = cv2.findHomography(px, fl, method=0)
    if H is None:
        raise ValueError(
            "findHomography failed. Usually the markers are collinear or nearly "
            "so -- spread them across the floor patch, not along one line."
        )
    H = np.asarray(H, dtype=np.float64)
    pred = _apply_h(H, px)
    err = np.linalg.norm(pred - fl, axis=1)
    return H, {
        "rms_m": float(np.sqrt(np.mean(err**2))),
        "max_m": float(np.max(err)),
        "n_points": int(px.shape[0]),
    }


def intrinsics_from_fov(width_px: int, height_px: int, hfov_rad: float) -> np.ndarray:
    """A square-pixel pinhole ``K`` from a horizontal field of view.

    For the **simulated** camera only. A real camera's ``K`` comes from a
    checkerboard, and a real robot's floor map comes from the homography.
    """
    fx = (width_px / 2.0) / math.tan(hfov_rad / 2.0)
    return np.array(
        [[fx, 0.0, width_px / 2.0], [0.0, fx, height_px / 2.0], [0.0, 0.0, 1.0]]
    )


def homography_from_pinhole(mount: CameraMount, K: np.ndarray) -> np.ndarray:
    """The exact pixel -> floor homography of an ideal pinhole camera.

    A homography *is* the projection of a plane through a pinhole, so this is
    not an approximation of the primary method -- it is the primary method's
    map, for a camera with no lens distortion. That is what makes the simulated
    pipeline faithful: the fake publisher turns floor metres into pixels with
    this, nav turns them back with a homography fitted to it, and any error
    that shows up is nav's.
    """
    A = mount.R.T  # cam_from_base rotation
    b = -A @ mount.t
    # Points on z = 0 in base: p_cam = A[:,0] x + A[:,1] y + b
    M = np.column_stack([A[:, 0], A[:, 1], b])
    H_pixel_from_floor = np.asarray(K, dtype=float) @ M
    return np.linalg.inv(H_pixel_from_floor)


def _apply_h(H: np.ndarray, pts: np.ndarray) -> np.ndarray:
    p = np.hstack([pts, np.ones((pts.shape[0], 1))])
    q = p @ H.T
    return q[:, :2] / q[:, 2:3]


# ---------------------------------------------------------------------------
# the calibration file
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class GroundCalibration:
    """What ``config/ground_homography.json`` holds.

    The mount geometry is recorded *in the file* so that loading it can assert
    the robot has not changed underneath it. That assertion is the entire
    reason the fields are duplicated here.
    """

    H_pixel_to_floor: np.ndarray
    frame_size: tuple[int, int]
    camera: dict[str, float]
    method: str = "homography"
    created: str = ""
    git_sha: str = ""
    residuals: dict[str, float] = field(default_factory=dict)
    points_pixel: list[list[float]] = field(default_factory=list)
    points_floor: list[list[float]] = field(default_factory=list)
    notes: str = ""

    def to_json(self) -> dict[str, Any]:
        return {
            "schema": CALIB_SCHEMA_VERSION,
            "method": self.method,
            "created": self.created or datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "git_sha": self.git_sha,
            "frame_size": list(self.frame_size),
            "camera": self.camera,
            "H_pixel_to_floor": [[float(v) for v in row] for row in self.H_pixel_to_floor],
            "residuals": self.residuals,
            "points": {"pixel": self.points_pixel, "floor": self.points_floor},
            "notes": self.notes,
        }

    def save(self, path: str | Path) -> None:
        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(self.to_json(), indent=2) + "\n", encoding="utf-8")

    @staticmethod
    def from_json(d: dict[str, Any], source: str = "<dict>") -> "GroundCalibration":
        if d.get("schema") != CALIB_SCHEMA_VERSION:
            raise GroundCalibrationError(
                f"{source}: calibration schema {d.get('schema')!r}, expected "
                f"{CALIB_SCHEMA_VERSION}. Re-run fodnav-calib-ground."
            )
        try:
            H = np.asarray(d["H_pixel_to_floor"], dtype=float).reshape(3, 3)
            size = tuple(int(v) for v in d["frame_size"])
        except (KeyError, ValueError, TypeError) as e:
            raise GroundCalibrationError(f"{source}: malformed calibration ({e})") from None
        pts = d.get("points", {}) or {}
        return GroundCalibration(
            H_pixel_to_floor=H,
            frame_size=(size[0], size[1]),
            camera=dict(d.get("camera", {})),
            method=str(d.get("method", "homography")),
            created=str(d.get("created", "")),
            git_sha=str(d.get("git_sha", "")),
            residuals=dict(d.get("residuals", {})),
            points_pixel=list(pts.get("pixel", [])),
            points_floor=list(pts.get("floor", [])),
            notes=str(d.get("notes", "")),
        )


def load_ground_calibration(
    path: str | Path, robot: Config | None = None
) -> GroundCalibration:
    """Load a calibration and assert it describes the robot in ``robot.yaml``.

    Skipping the assertion is not an option worth offering. A calibration from
    before the mount was re-printed produces plausible numbers that are wrong
    by a fixed amount, which reads as a controller problem for as long as
    anyone is willing to keep tuning gains.
    """
    p = Path(path)
    if not p.is_file():
        raise GroundCalibrationError(
            f"{p}: no ground calibration. Run:\n"
            f"    fodnav-calib-ground --help\n"
            f"Nothing vision-driven works until the floor projection exists."
        )
    calib = GroundCalibration.from_json(json.loads(p.read_text(encoding="utf-8")), str(p))
    if robot is not None:
        _assert_matches_robot(calib, robot, str(p))
    return calib


def _assert_matches_robot(calib: GroundCalibration, robot: Config, source: str) -> None:
    problems: list[str] = []

    want_size = (robot.get("camera.capture_width_px"), robot.get("camera.capture_height_px"))
    if tuple(calib.frame_size) != want_size:
        problems.append(
            f"  capture resolution: calibration {tuple(calib.frame_size)}, "
            f"robot.yaml {want_size}"
        )
    for key, tol in _GEOMETRY_TOL.items():
        if key not in calib.camera:
            problems.append(f"  {key}: not recorded in the calibration file")
            continue
        want = robot.get(f"camera.{key}")
        got = float(calib.camera[key])
        if abs(got - want) > tol:
            problems.append(f"  camera.{key}: calibration {got}, robot.yaml {want}")

    if problems:
        raise GroundCalibrationError(
            f"{source} was captured on a different camera geometry than "
            f"{robot.source} describes:\n"
            + "\n".join(problems)
            + "\n\nEither the mount moved, or someone re-measured, or this file is "
            "from another robot. A calibration is valid only for the geometry it "
            "was taken at. Re-run fodnav-calib-ground."
        )


# ---------------------------------------------------------------------------
# projection
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class GroundPoint:
    """A detection's floor position in ``base``, with its polar form."""

    x: float
    y: float

    @property
    def range_m(self) -> float:
        return math.hypot(self.x, self.y)

    @property
    def bearing_rad(self) -> float:
        return math.atan2(self.y, self.x)

    @property
    def xy(self) -> tuple[float, float]:
        return (self.x, self.y)


class GroundProjector:
    """Turns pixels into floor points in ``base``, or refuses to.

    ``max_valid_range_m`` comes from ``camera.fov_far_limit_m`` -- the furthest
    distance the detector still fires reliably -- rather than being a second
    number someone types in. Beyond it the calibration is extrapolating and the
    projection degrades fast.
    """

    def __init__(
        self,
        calib: GroundCalibration,
        max_valid_range_m: float,
        min_range_m: float = 0.0,
    ) -> None:
        self.calib = calib
        self.H = np.asarray(calib.H_pixel_to_floor, dtype=float)
        self.H_inv = np.linalg.inv(self.H)
        self.frame_size = tuple(calib.frame_size)
        self.max_valid_range_m = float(max_valid_range_m)
        self.min_range_m = float(min_range_m)
        # Which side of the horizon the floor is on, in this homography's
        # homogeneous coordinate. Taken from the calibration points if they are
        # recorded (they are, unless someone hand-wrote the file), otherwise
        # from the bottom-centre pixel, which is floor by construction.
        self._w_sign = self._floor_side_sign()
        self.n_rejected_horizon = 0
        self.n_rejected_range = 0

    def _floor_side_sign(self) -> float:
        pts = self.calib.points_pixel
        if not pts:
            pts = [[self.frame_size[0] / 2.0, self.frame_size[1] - 1.0]]
        p = np.asarray(pts, dtype=float).reshape(-1, 2)
        w = np.hstack([p, np.ones((p.shape[0], 1))]) @ self.H[2]
        s = float(np.sign(np.mean(w)))
        return s if s != 0.0 else 1.0

    # -- the frame-size assertion --------------------------------------

    def check_frame_size(self, frame_size: Sequence[int]) -> None:
        """Assert a detection message was captured at the calibrated size.

        CLAUDE.md section 8 makes this an assertion, not a warning: if the
        resolution does not match, the homography is invalid and every
        projection is silently wrong.
        """
        if tuple(int(v) for v in frame_size) != tuple(self.frame_size):
            raise GroundCalibrationError(
                f"detections arrived at frame_size {tuple(frame_size)} but the ground "
                f"calibration was captured at {tuple(self.frame_size)}. Every projection "
                f"would be silently wrong. Either the camera capture resolution changed "
                f"(and the calibration must be redone) or the publisher is sending "
                f"letterboxed network-input coordinates instead of original-frame pixels."
            )

    # -- pixels in, metres out ------------------------------------------

    def project_pixel(self, u: float, v: float) -> GroundPoint | None:
        """Project one pixel to the floor, or ``None`` if it cannot be.

        ``None`` means one of: above the horizon, behind the robot, closer than
        the near gate, or further than the calibration supports. Callers treat
        ``None`` as "no detection", never as zero.
        """
        w = self.H[2, 0] * u + self.H[2, 1] * v + self.H[2, 2]
        # Horizon guard. A pixel on the far side of the vanishing line images a
        # ray that never descends to the floor; its "intersection" is behind
        # the camera and the sign of w is what says so.
        if w * self._w_sign <= 1e-12:
            self.n_rejected_horizon += 1
            return None
        x = (self.H[0, 0] * u + self.H[0, 1] * v + self.H[0, 2]) / w
        y = (self.H[1, 0] * u + self.H[1, 1] * v + self.H[1, 2]) / w
        r = math.hypot(x, y)
        if x <= 0.0:
            self.n_rejected_horizon += 1
            return None
        if r > self.max_valid_range_m or r < self.min_range_m:
            self.n_rejected_range += 1
            return None
        return GroundPoint(x, y)

    def project_detection(self, det, frame_size: Sequence[int] | None = None) -> GroundPoint | None:
        """Project a :class:`~fodnav.link.detections.Detection`'s ground point.

        The ground point of a detection is the **bottom-centre** of its box.
        """
        if frame_size is not None:
            self.check_frame_size(frame_size)
        u, v = det.ground_px
        return self.project_pixel(u, v)

    def pixel_from_floor(self, x: float, y: float) -> tuple[float, float]:
        """Inverse map, for drawing overlays and for the simulated camera."""
        q = self.H_inv @ np.array([x, y, 1.0])
        if abs(q[2]) < 1e-12:
            raise ValueError(f"floor point ({x}, {y}) has no image under this calibration")
        return (float(q[0] / q[2]), float(q[1] / q[2]))

    def visible(self, x: float, y: float) -> bool:
        """Would a floor point land inside the captured frame?"""
        try:
            u, v = self.pixel_from_floor(x, y)
        except ValueError:
            return False
        return 0.0 <= u < self.frame_size[0] and 0.0 <= v < self.frame_size[1]

    def stats(self) -> dict[str, int]:
        return {
            "rejected_horizon": self.n_rejected_horizon,
            "rejected_range": self.n_rejected_range,
        }


def projector_from_config(
    robot: Config, calib_path: str | Path, min_range_m: float = 0.0
) -> GroundProjector:
    """Build a projector, asserting the calibration matches the robot."""
    (far,) = robot.require("camera.fov_far_limit_m", needed_by="the ground projection range gate")
    calib = load_ground_calibration(calib_path, robot)
    return GroundProjector(calib, max_valid_range_m=far, min_range_m=min_range_m)


def mount_from_config(robot: Config) -> CameraMount:
    """The measured ``base_T_cam`` transform. Raises if the mount is unmeasured."""
    h, tilt, ox, oy, roll = robot.require(
        "camera.height_m",
        "camera.tilt_rad",
        "camera.offset_x_m",
        "camera.offset_y_m",
        "camera.roll_rad",
        needed_by="the camera mount transform",
    )
    return CameraMount(height_m=h, tilt_rad=tilt, offset_x_m=ox, offset_y_m=oy, roll_rad=roll)
