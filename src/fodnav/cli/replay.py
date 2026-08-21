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
from ..link.detections import ReplayDetectionSource, iter_jsonl, select_targets
from ..target import TargetTracker, TrackerParams
from ._common import add_config_args, die, load_configs


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="fodnav-replay", description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    add_config_args(p, default_robot="config/sim_robot.yaml")
    p.add_argument("log", help="a detections.jsonl from a run, or from tools/fake_detections.py")
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
    source = ReplayDetectionSource(args.log, speed=args.speed, rebase_capture_time=True)
    source.start()

    tracker = TargetTracker(TrackerParams.from_config(nav))
    targets = nav.get("detections.target_classes")
    ignored = nav.get("detections.ignore_classes")
    drop_conf = nav.get("detections.drop_conf")

    classes: Counter[str] = Counter()
    n_frames = n_dets = n_kept = n_projected = 0
    n_empty = 0
    ranges: list[float] = []
    confirmed_ids: set[int] = set()
    sizes: Counter[tuple[int, int]] = Counter()
    t = 0.0

    while not source.finished:
        for frame in source.poll():
            n_frames += 1
            sizes[frame.frame_size] += 1
            if not frame.dets:
                n_empty += 1
            for d in frame.dets:
                classes[d.cls] += 1
            n_dets += len(frame.dets)
            kept = select_targets(frame, targets, ignored, drop_conf)
            n_kept += len(kept)
            observations = []
            for d in kept:
                if tuple(frame.frame_size) != tuple(projector.frame_size):
                    continue
                g = projector.project_detection(d)
                if g is None:
                    continue
                n_projected += 1
                ranges.append(g.range_m)
                observations.append((g, d.conf, d.cls))
            t += 1.0 / 30.0
            for tr in tracker.update(observations, t):
                confirmed_ids.add(tr.id)
            if args.limit and n_frames >= args.limit:
                source._i = len(source._records)  # noqa: SLF001
                break

    stats = projector.stats()
    summary = {
        "log": args.log,
        "records": n_records,
        "messages_parsed": n_frames,
        "malformed": source.stats.malformed,
        "empty_scenes": n_empty,
        "frame_sizes": {f"{w}x{h}": n for (w, h), n in sizes.items()},
        "detections": n_dets,
        "by_class": dict(classes),
        "kept_as_targets": n_kept,
        "projected": n_projected,
        "rejected_horizon": stats["rejected_horizon"],
        "rejected_range": stats["rejected_range"],
        "confirmed_tracks": len(confirmed_ids),
        "range_m": {
            "min": round(min(ranges), 3) if ranges else None,
            "max": round(max(ranges), 3) if ranges else None,
            "median": round(sorted(ranges)[len(ranges) // 2], 3) if ranges else None,
        },
    }

    if args.json:
        print(json.dumps(summary, indent=2))
        return 0

    print(f"{args.log}: {n_records} records, {n_frames} parsed, "
          f"{source.stats.malformed} malformed")
    print(f"  frame sizes      {summary['frame_sizes']} (calibrated at "
          f"{projector.frame_size[0]}x{projector.frame_size[1]})")
    for size in sizes:
        if tuple(size) != tuple(projector.frame_size):
            print(f"  !! {size[0]}x{size[1]} does not match the calibration -- every "
                  f"projection from those frames would be silently wrong")
    print(f"  detections       {n_dets} in {n_frames} messages "
          f"({n_empty} empty scenes, which are reports, not silence)")
    for cls, n in classes.most_common():
        mark = "  <- ignored" if cls.lower() in {c.lower() for c in ignored} else ""
        print(f"      {cls:12s} {n:6d}{mark}")
    print(f"  kept as targets  {n_kept}")
    print(f"  projected        {n_projected}")
    print(f"      rejected: {stats['rejected_horizon']} above the horizon or behind, "
          f"{stats['rejected_range']} out of range")
    if ranges:
        r = summary["range_m"]
        print(f"      range: {r['min']} to {r['max']} m, median {r['median']} m")
    print(f"  confirmed tracks {len(confirmed_ids)}")
    if n_dets and not n_projected:
        print("\n  Nothing projected. Either the frame size does not match the "
              "calibration,\n  or every detection fell outside the calibrated patch.",
              file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
