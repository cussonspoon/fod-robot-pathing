"""A camera that does not exist, for a robot that does not exist.

Its whole job is to make the vision-driven half of the stack runnable on a
laptop: it turns floor positions in ``base`` into bounding boxes in pixels, so
``tools/fake_detections.py`` can say "put a bolt at (0.8, 0.15)" and publish
something shaped like what Bthcorn's publisher will eventually send.

The optics are invented -- a wide-angle module at the mount geometry in
``config/sim_robot.yaml`` -- and nothing here is a measurement of the real
camera. What is *not* invented is the geometry: this is an exact pinhole
projection through the same :class:`~fodnav.frames.CameraMount` the real
projection uses, and a pinhole's view of a plane is exactly a homography. So
the loop closes honestly: this synthesises pixels from floor metres, nav maps
them back with a homography fitted to those same correspondences, and anything
that does not round-trip is a bug in nav rather than an artefact of the fake.

It also reproduces the one perception detail that bites hardest: a box's
**bottom edge** is where the object touches the floor and its centroid is not.
Objects here have height, so a consumer that projects the centroid gets a
range error proportional to that height -- in simulation, where it is cheap to
notice.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from ..config import Config
from ..frames import CameraMount
from ..ground import (
    GroundCalibration,
    GroundProjector,
    fit_homography,
    homography_from_pinhole,
    intrinsics_from_fov,
    mount_from_config,
)

__all__ = ["SimCamera", "DEFAULT_HFOV_DEG"]

#: Horizontal field of view of the fictional lens, degrees. A wide module, of
#: the sort you would actually point at the floor 22 cm in front of a robot.
#: If the real camera turns out to be narrower, the real ``fov_near_limit_m``
#: measurement will say so and the fiction is irrelevant.
DEFAULT_HFOV_DEG = 102.0


@dataclass
class SimObject:
    """Something on the floor with a physical size, so it images like an object."""

    x: float
    y: float = 0.0
    cls: str = "bolt"
    width_m: float = 0.012
    length_m: float = 0.060
    height_m: float = 0.008


class SimCamera:
    def __init__(self, robot: Config, hfov_deg: float = DEFAULT_HFOV_DEG) -> None:
        self.mount: CameraMount = mount_from_config(robot)
        self.width_px, self.height_px = robot.require(
            "camera.capture_width_px", "camera.capture_height_px",
            needed_by="the simulated camera",
        )
        self.hfov_rad = math.radians(hfov_deg)
        self.K = intrinsics_from_fov(self.width_px, self.height_px, self.hfov_rad)
        self.H_pixel_to_floor = homography_from_pinhole(self.mount, self.K)
        self._cam_from_base_R = self.mount.R.T
        self._cam_from_base_t = -self._cam_from_base_R @ self.mount.t

    # -- projection -----------------------------------------------------

    def project_point(self, p_base) -> tuple[float, float] | None:
        """Full 3D pinhole projection of a point in ``base``. ``None`` if behind."""
        p = np.asarray(p_base, dtype=float)
        p_cam = self._cam_from_base_R @ p + self._cam_from_base_t
        if p_cam[2] <= 1e-6:  # behind the lens
            return None
        q = self.K @ p_cam
        return (float(q[0] / q[2]), float(q[1] / q[2]))

    def in_frame(self, uv: tuple[float, float] | None) -> bool:
        if uv is None:
            return False
        u, v = uv
        return 0.0 <= u < self.width_px and 0.0 <= v < self.height_px

    def bbox_for(self, obj: SimObject) -> tuple[float, float, float, float] | None:
        """Image an object as ``[x, y, w, h]`` in original-frame pixels.

        The returned box's bottom edge is the object's contact with the floor,
        because that is what a detector's box does and it is what
        ``Detection.ground_px`` assumes.

        ``None`` if the object's contact point is not in frame -- which is
        exactly what happens as the robot closes on it, and is the reason the
        terminal blind leg exists (CLAUDE.md section 5).
        """
        hw, hl, hh = obj.width_m / 2.0, obj.length_m / 2.0, obj.height_m
        corners = [
            (obj.x + sx * hl, obj.y + sy * hw, z)
            for sx in (-1, 1)
            for sy in (-1, 1)
            for z in (0.0, hh)
        ]
        uvs = [self.project_point(np.array(c)) for c in corners]
        if any(p is None for p in uvs):
            return None
        us = [p[0] for p in uvs]
        vs = [p[1] for p in uvs]
        # The contact point must be in frame; a box whose bottom edge has left
        # the image tells nav nothing about where the object is on the floor.
        contact = self.project_point(np.array([obj.x, obj.y, 0.0]))
        if not self.in_frame(contact):
            return None
        x0, x1 = min(us), max(us)
        y0, y1 = min(vs), max(vs)
        if x1 - x0 < 1.0:
            x0, x1 = x0 - 0.5, x1 + 0.5
        if y1 - y0 < 1.0:
            y0, y1 = y0 - 0.5, y1 + 0.5
        return (x0, y0, x1 - x0, y1 - y0)

    # -- measurements of the fiction ------------------------------------
    #
    # The numbers in sim_robot.yaml's camera block are *derived from this
    # model*, not chosen independently. A fictional robot whose declared field
    # of view disagrees with the camera it actually publishes through would
    # make the blind-leg handover trigger at the wrong distance and teach us
    # something false.

    def floor_x_at_image_row(self, v: float) -> float | None:
        """Forward distance in ``base`` of the floor point imaged on row ``v``."""
        proj = self.projector(max_valid_range_m=1e6)
        return None if (g := proj.project_pixel(self.width_px / 2.0, v)) is None else g.x

    def near_limit_m(self) -> float:
        """Closest floor distance still in frame: the bottom image row."""
        x = self.floor_x_at_image_row(self.height_px - 0.5)
        if x is None:
            raise RuntimeError("the simulated camera cannot see the floor at all")
        return x

    def horizon_row(self) -> float | None:
        """Image row of the horizon, or ``None`` if it is above the frame."""
        # The horizon is where the plane-induced homography's denominator is 0.
        H = self.H_pixel_to_floor
        u = self.width_px / 2.0
        if abs(H[2, 1]) < 1e-15:
            return None
        v = -(H[2, 0] * u + H[2, 2]) / H[2, 1]
        return v if 0.0 <= v < self.height_px else None

    def width_at(self, x_m: float) -> float:
        """Ground width of the frame at forward distance ``x_m``.

        One of the two defensible swath widths in CLAUDE.md section 9 -- the
        one that applies if coverage means *detection* rather than collection.
        """
        proj = self.projector(max_valid_range_m=1e6)
        lo, hi = 0.0, 10.0
        for _ in range(60):  # bisect on "is (x, y) still inside the frame"
            mid = 0.5 * (lo + hi)
            try:
                u, _v = proj.pixel_from_floor(x_m, mid)
                inside = 0.0 <= u < self.width_px
            except ValueError:
                inside = False
            lo, hi = (mid, hi) if inside else (lo, mid)
        return 2.0 * lo

    # -- calibration ----------------------------------------------------

    def marker_grid(
        self, x_range: tuple[float, float] = (0.35, 1.20), y_half: float = 0.35, n: int = 3
    ) -> tuple[list[list[float]], list[list[float]]]:
        """Synthetic floor markers, as if taped down and clicked in one frame.

        Spread across the patch rather than along a line, because collinear
        markers are the usual reason ``findHomography`` returns something
        useless.
        """
        pixel: list[list[float]] = []
        floor: list[list[float]] = []
        xs = np.linspace(x_range[0], x_range[1], n)
        ys = np.linspace(-y_half, y_half, n)
        for x in xs:
            for y in ys:
                uv = self.project_point(np.array([float(x), float(y), 0.0]))
                if not self.in_frame(uv):
                    continue
                pixel.append([uv[0], uv[1]])
                floor.append([float(x), float(y)])
        if len(pixel) < 6:
            raise RuntimeError(
                f"only {len(pixel)} synthetic markers landed in frame; widen the "
                f"grid or check the mount geometry in the config"
            )
        return pixel, floor

    def calibration(self, fit: bool = True, notes: str = "") -> GroundCalibration:
        """A ground calibration for the fictional camera.

        ``fit=True`` runs the real ``findHomography`` path over synthetic
        markers, so the simulated stack exercises the same code the afternoon
        with the tape measure will. ``fit=False`` writes the analytic
        homography, which is only useful for isolating a fitting bug.
        """
        pixel, floor = self.marker_grid()
        if fit:
            H, residuals = fit_homography(pixel, floor)
        else:
            H, residuals = self.H_pixel_to_floor, {"rms_m": 0.0, "max_m": 0.0, "n_points": 0}
        return GroundCalibration(
            H_pixel_to_floor=H,
            frame_size=(self.width_px, self.height_px),
            camera={
                "height_m": self.mount.height_m,
                "tilt_deg": math.degrees(self.mount.tilt_rad),
                "offset_x_m": self.mount.offset_x_m,
                "offset_y_m": self.mount.offset_y_m,
                "roll_deg": math.degrees(self.mount.roll_rad),
            },
            method="synthetic" if fit else "synthetic-analytic",
            residuals=residuals,
            points_pixel=pixel,
            points_floor=floor,
            notes=notes
            or (
                "SYNTHETIC. Fitted to a fictional pinhole camera "
                f"(hfov {math.degrees(self.hfov_rad):.1f} deg) over config/sim_robot.yaml. "
                "Not a calibration of any physical camera."
            ),
        )

    def projector(self, max_valid_range_m: float, min_range_m: float = 0.0) -> GroundProjector:
        return GroundProjector(
            self.calibration(fit=False), max_valid_range_m, min_range_m=min_range_m
        )
