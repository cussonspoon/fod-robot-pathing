"""``fodnav-replay`` -- feed a recorded detection log back through the stack.

Every run appends every detection message it received to a JSONL log. This
reads one back through the *same* :class:`~fodnav.link.detections` interface a
live broker feeds, so the whole nav stack is testable against real vision
output with no camera, no Pi and no robot.

Two modes. The default analyses: it runs parse, class filtering, projection and
association over the log and reports what nav would have seen -- how many
messages were malformed, how many detections were rejected by the horizon or
range gates, how many tracks were confirmed, how far away they were. That is
usually the question ("why did it not see the nail?").

``--drive`` additionally runs the controllers against a simulated chassis. Be
clear about what that shows: the recorded boxes were taken from wherever the
real robot was at the time, and the simulated robot will not go there, so the
geometry stops being consistent the moment it moves. It exercises the code
paths; it does not reproduce the run.
"""

from __future__ import annotations

import argparse
import json
import sys
from collections import Counter

from ..config import ConfigError
from ..ground import load_ground_calibration, GroundProjector
from ..frames import Pose2D
from ..link.vision import CONFIRM, PICK, ReplayVisionSource, iter_jsonl, select_targets
from ..target import TargetParams, TargetSet

from ._common import add_config_args, die, load_configs


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="fodnav-replay", description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    add_config_args(p, default_robot="config/sim_robot.yaml")
    p.add_argument("log", help="a detections.jsonl from a run, or from tools/fake_vision_log.py")
    p.add_argument("--ground-calib", default=None,
                   help="calibration to project with (default: sim_ground_homography.json "
                        "when the robot config is the fictional one)")
    p.add_argument("--speed", type=float, default=0.0,
                   help="0 replays as fast as possible; 1 replays in real time")
    p.add_argument("--json", action="store_true")
    p.add_argument("--limit", type=int, default=0, help="stop after this many messages")
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        robot, nav = load_configs(args)
    except ConfigError as e:
        die(str(e))

    calib_path = args.ground_calib or (
        "config/sim_ground_homography.json"
        if "sim_robot" in robot.source
        else "config/ground_homography.json"
    )
    try:
        calib = load_ground_calibration(calib_path, robot)
    except Exception as e:
        die(f"{e}")
    projector = GroundProjector(calib, max_valid_range_m=robot.get("camera.fov_far_limit_m"))

    n_records = sum(1 for _ in iter_jsonl(args.log))
    source = ReplayVisionSource(args.log, speed=args.speed)
    source.start()

    tracker = TargetSet(TargetParams.from_config(nav))
    states, sizes = Counter(), Counter()
    n_frames = n_targets = n_kept = n_projected = n_empty = n_errors = 0
    ranges: list[float] = []
    ids: set[int] = set()
    first_error = ""
    t = 0.0

    while not source.finished:
        for frame in source.poll():
            n_frames += 1
            sizes[frame.frame_size] += 1
            if frame.error:
                n_errors += 1
                first_error = first_error or frame.error
                continue
            if not frame.targets:
                n_empty += 1
            n_targets += len(frame.targets)
            for tgt in frame.targets:
                states[tgt.state] += 1
            kept = select_targets(frame, states=(CONFIRM,), actions=(PICK,))
            n_kept += len(kept)
            if tuple(frame.frame_size) != tuple(projector.frame_size):
                continue
            t += 1.0 / 30.0
            for tr in tracker.update(frame, projector, Pose2D(), t):
                ids.add(tr.id)
                ranges.append(tr.range_m)
                n_projected += 1

    stats = projector.stats()
    summary = {
        "log": args.log,
        "records": n_records,
        "frames": n_frames,
        "error_frames": n_errors,
        "first_error": first_error,
        "empty_frames": n_empty,
        "frame_sizes": {f"{w}x{h}": n for (w, h), n in sizes.items()},
        "targets": n_targets,
        "by_state": dict(states),
        "confirmed_and_pickable": n_kept,
        "projected": n_projected,
        "rejected_horizon": stats["rejected_horizon"],
        "rejected_range": stats["rejected_range"],
        "distinct_track_ids": len(ids),
        "range_m": {
            "min": round(min(ranges), 3) if ranges else None,
            "max": round(max(ranges), 3) if ranges else None,
            "median": round(sorted(ranges)[len(ranges) // 2], 3) if ranges else None,
        },
    }

    if args.json:
        print(json.dumps(summary, indent=2))
        return 0

    print(f"{args.log}: {n_records} records, {n_frames} frames, {n_errors} carrying an error")
    if first_error:
        print(f"  first error      {first_error}")
    print(f"  frame sizes      {summary['frame_sizes']} "
          f"(calibrated at {projector.frame_size[0]}x{projector.frame_size[1]})")
    for size in sizes:
        if tuple(size) != tuple(projector.frame_size):
            print(f"  !! {size[0]}x{size[1]} does not match the calibration -- every "
                  f"projection from those frames would be silently wrong")
    print(f"  targets          {n_targets} over {n_frames} frames "
          f"({n_empty} empty, which mean the floor is clear, not that vision died)")
    for state, n in states.most_common():
        mark = "  <- chased" if state == CONFIRM else ""
        print(f"      {state:10s} {n:6d}{mark}")
    print(f"  confirmed+PICK   {n_kept}")
    print(f"  projected        {n_projected} across {len(ids)} distinct track ids")
    print(f"      rejected: {stats['rejected_horizon']} above the horizon or behind, "
          f"{stats['rejected_range']} out of range")
    if ranges:
        r = summary["range_m"]
        print(f"      range: {r['min']} to {r['max']} m, median {r['median']} m")
    if n_targets and not n_projected:
        print("\n  Nothing projected. Either the frame size does not match the "
              "calibration,\n  or every detection fell outside the calibrated patch.",
              file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
