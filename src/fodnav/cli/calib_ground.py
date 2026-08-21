"""``fodnav-calib-ground`` -- solve the pixel-to-floor homography.

The primary method from CLAUDE.md section 7: tape at least six markers on the
floor at known positions in ``base``, capture one frame, read off their pixel
positions, and solve the 3x3 ``H`` that maps image pixels to floor metres. No
camera intrinsics needed, and it absorbs lens distortion approximately over the
calibrated region. One afternoon of work.

**Reading the pixel positions is not done here.** The runtime dependency is
``opencv-python-headless`` -- deliberately, since the only OpenCV call in the
runtime is ``findHomography`` and the GUI build drags in X11 for nothing -- so
there is no window to click in. Get the pixel coordinates however you like
(any image viewer will show them) and pass them in as a small JSON or CSV file.
The correspondences are the calibration record; the clicking is not.

Input format, JSON::

    {"points": [{"pixel": [1102, 812], "floor": [0.60, -0.30]}, ...]}

or CSV with a header ``u,v,x,y``. Floor coordinates are metres in ``base``:
+x forward from the axle midpoint, +y left.

Check the residuals it prints. A 3 cm RMS is not a calibration, it is a warning.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

from ..config import ConfigError
from ..ground import GroundCalibration, fit_homography
from ..runlog import git_describe
from ..sim.camera import DEFAULT_HFOV_DEG, SimCamera
from ._common import add_config_args, die, load_configs


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="fodnav-calib-ground",
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    add_config_args(p)
    p.add_argument("--points", default=None,
                   help="JSON or CSV of pixel/floor correspondences (six or more)")
    p.add_argument("--synthetic", action="store_true",
                   help="calibrate the fictional camera in sim/camera.py instead. "
                        "Not a calibration of any physical camera.")
    p.add_argument("--hfov-deg", type=float, default=DEFAULT_HFOV_DEG,
                   help="synthetic only: the fictional lens")
    p.add_argument("-o", "--output", default=None,
                   help="where to write it (default config/ground_homography.json, "
                        "or config/sim_ground_homography.json for --synthetic)")
    p.add_argument("--notes", default="", help="recorded in the file: tape layout, lighting, who")
    p.add_argument("--force", action="store_true", help="overwrite an existing calibration")
    return p


def load_points(path: str) -> tuple[list[list[float]], list[list[float]]]:
    p = Path(path)
    if not p.is_file():
        die(f"{path}: no such file")
    pixel: list[list[float]] = []
    floor: list[list[float]] = []
    if p.suffix.lower() == ".csv":
        with p.open(newline="", encoding="utf-8") as fh:
            for row in csv.DictReader(fh):
                try:
                    pixel.append([float(row["u"]), float(row["v"])])
                    floor.append([float(row["x"]), float(row["y"])])
                except (KeyError, ValueError) as e:
                    die(f"{path}: expected columns u,v,x,y -- {e}")
    else:
        try:
            data = json.loads(p.read_text(encoding="utf-8"))
        except json.JSONDecodeError as e:
            die(f"{path}: {e}")
        for i, entry in enumerate(data.get("points", [])):
            try:
                pixel.append([float(entry["pixel"][0]), float(entry["pixel"][1])])
                floor.append([float(entry["floor"][0]), float(entry["floor"][1])])
            except (KeyError, IndexError, TypeError, ValueError) as e:
                die(f"{path}: point {i} is malformed -- {e}")
    if len(pixel) < 6:
        die(
            f"{path}: {len(pixel)} correspondences. Four is the mathematical minimum "
            f"and gives an exact fit with no residual to look at, which is "
            f"indistinguishable from a perfect one. Tape down at least six."
        )
    return pixel, floor


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        robot, nav = load_configs(args)
    except ConfigError as e:
        die(str(e))

    if args.synthetic:
        cam = SimCamera(robot, hfov_deg=args.hfov_deg)
        calib = cam.calibration(notes=args.notes)
        default_out = "config/sim_ground_homography.json"
    else:
        if not args.points:
            die("give --points FILE, or --synthetic for the fictional camera")
        pixel, floor = load_points(args.points)
        H, residuals = fit_homography(pixel, floor)
        try:
            camera = {
                "height_m": robot.get("camera.height_m"),
                "tilt_deg": robot.get("camera.tilt_deg"),
                "offset_x_m": robot.get("camera.offset_x_m"),
                "offset_y_m": robot.get("camera.offset_y_m"),
                "roll_deg": robot.get("camera.roll_deg"),
            }
            size = (robot.get("camera.capture_width_px"), robot.get("camera.capture_height_px"))
        except ConfigError as e:
            die(
                f"{e}\n\nThe mount geometry has to be recorded *in* the calibration so "
                f"that loading it can assert the robot has not changed underneath it."
            )
        calib = GroundCalibration(
            H_pixel_to_floor=H,
            frame_size=size,
            camera=camera,
            method="homography",
            git_sha=git_describe().get("sha", ""),
            residuals=residuals,
            points_pixel=pixel,
            points_floor=floor,
            notes=args.notes,
        )
        default_out = "config/ground_homography.json"

    out = Path(args.output or default_out)
    if out.exists() and not args.force:
        die(f"{out} exists. Pass --force to replace it (and commit the old one first).")
    calib.save(out)

    print(f"wrote {out}")
    print(f"  method     {calib.method}")
    print(f"  frame      {calib.frame_size[0]} x {calib.frame_size[1]}")
    print(f"  camera     h={calib.camera['height_m']} m, tilt={calib.camera['tilt_deg']} deg")
    r = calib.residuals
    if r.get("n_points"):
        print(f"  residuals  rms {r['rms_m'] * 1000:.1f} mm, worst {r['max_m'] * 1000:.1f} mm, "
              f"over {int(r['n_points'])} points")
        if r["rms_m"] > 0.02:
            print("\n  WARNING: an RMS over 2 cm is not a calibration, it is a warning.")
            print("  Check the markers are not nearly collinear, that the floor positions")
            print("  are measured from the axle midpoint, and that the mount has not moved.")
    print("\nThis calibration is valid only for the mount geometry above. If the mount")
    print("moves, is loosened, knocked or re-printed, it is void and must be redone.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
