#!/usr/bin/env python3
"""Write a recorded-looking vision log without a camera, a Pi or a robot.

This replaces ``tools/fake_detections.py``, which published an MQTT schema that
turned out never to exist -- the vision side is a library you import, not a
topic you subscribe to (docs/vendor/fod-vision-v0.3.0-INTEGRATION.md).

What it emits is his ``detail()`` dictionary, one JSON object per line, which is
exactly the format ``fodnav-replay`` consumes and exactly the format a real run
records. So a log made here and a log made on the robot are interchangeable, and
nothing downstream can tell them apart -- which is the point.

It is still a stand-in written from his spec, not his code. The day someone
records thirty seconds of real detector output, use that instead: it is the only
thing that will tell you whether our projection agrees with his boxes.

    python tools/fake_vision_log.py --scenario approach -o logs/fake.jsonl
    uv run fodnav-replay logs/fake.jsonl
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from fodnav.config import load_robot_config  # noqa: E402
from fodnav.frames import Pose2D  # noqa: E402
from fodnav.sim.camera import SimCamera  # noqa: E402
from fodnav.sim.scene import SceneNoise, SimScene  # noqa: E402


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--scenario", choices=("approach", "static", "empty", "dropout"),
                    default="approach")
    ap.add_argument("--x", type=float, default=1.10, help="target x in base, metres")
    ap.add_argument("--y", type=float, default=0.10, help="target y in base, metres (left)")
    ap.add_argument("--speed", type=float, default=0.20, help="approach speed, m/s")
    ap.add_argument("--seconds", type=float, default=6.0)
    ap.add_argument("--rate", type=float, default=30.0, help="his camera cadence")
    ap.add_argument("--miss-rate", type=float, default=0.0)
    ap.add_argument("--jitter-px", type=float, default=1.0)
    ap.add_argument("--dropout-after", type=float, default=3.0,
                    help="dropout scenario: emit an error frame after this long")
    ap.add_argument("--robot-config", default="config/sim_robot.yaml")
    ap.add_argument("-o", "--output", default="-", help="JSONL path, or - for stdout")
    args = ap.parse_args(argv)

    robot = load_robot_config(args.robot_config)
    cam = SimCamera(robot)
    scene = SimScene(cam, noise=SceneNoise(miss_rate=args.miss_rate, jitter_px=args.jitter_px))
    if args.scenario != "empty":
        scene.add(args.x, args.y)

    out = sys.stdout if args.output == "-" else open(args.output, "w", encoding="utf-8")
    n = 0
    try:
        for i in range(int(args.seconds * args.rate)):
            t = i / args.rate
            if args.scenario == "dropout" and t >= args.dropout_after:
                # His semantics: a dead camera raises on read, and detail()
                # carries the reason. Not an empty frame -- that would mean
                # "the floor is clear", which is the one thing it must not say.
                detail = {"frame_id": i, "age": t - args.dropout_after, "blocked": False,
                          "camera": {"frame_size": [cam.width_px, cam.height_px]},
                          "tracks": [], "error": "RuntimeError('capture thread died')"}
            else:
                # The robot closes on a fixed target, so move the robot rather
                # than the object -- the object leaving the frame near the end
                # is the terminal blind leg, and it should be in the log.
                driven = args.speed * t if args.scenario == "approach" else 0.0
                detail = scene.render(Pose2D(driven, 0.0, 0.0), t)
                detail["age"] = 0.0
            out.write(json.dumps(detail, separators=(",", ":")) + "\n")
            n += 1
    finally:
        if out is not sys.stdout:
            out.close()
    print(f"wrote {n} frames to {args.output}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
