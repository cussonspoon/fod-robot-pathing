"""The vision link against **real** fod-vision output, not our reading of the spec.

The fixtures are twelve-line slices of Bthcorn's 2026-09-24 captures
(``cv_tests/2026-09-24/data/``, from his ``scripts/capture_raw.py`` on the Pi
with the Hailo). Each line is a ``detail()`` dict plus two fields his capture
script adds: ``t``, and ``targets`` -- *his* ``latest()`` for the same frame.

``parse_detail`` never reads ``targets``; it rebuilds ``latest()`` from
``tracks`` by dropping coasting ones. These tests hold that reconstruction to
his answer, which is the one thing a spec-derived fixture could never check.

* ``real_detail_n640.jsonl`` -- frames 2-16 of n640-c: the start-up transient,
  including a near-whole-frame box that reaches CONFIRM, and coasting tracks.
* ``real_detail_s640.jsonl`` -- lines 107-118 of s640-c: mid-run, several
  confirmed targets at once, coasting tracks.

The full-capture check is ``tools/audit_cv_capture.py``; results are in
``cv_tests/2026-09-24/output/``.
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from fodnav.config import load_robot_config
from fodnav.frames import Pose2D
from fodnav.ground import GroundProjector, load_ground_calibration
from fodnav.link.vision import CONFIRM, PICK, ReplayVisionSource, iter_jsonl, parse_detail, select_targets
from fodnav.target import TargetSet

FIXTURES = Path(__file__).parent / "fixtures"
REAL = sorted(FIXTURES.glob("real_detail_*.jsonl"))


def records(path: Path) -> list[dict]:
    return list(iter_jsonl(path))


@pytest.fixture(params=REAL, ids=lambda p: p.stem)
def real_log(request) -> Path:
    return request.param


def test_the_fixtures_exist():
    assert len(REAL) == 2


def test_every_real_line_parses(real_log):
    recs = records(real_log)
    assert len(recs) == sum(1 for line in real_log.open() if line.strip())
    for d in recs:
        f = parse_detail(d, t_recv=0.0)
        assert f.ok
        assert f.frame_size == (1280, 720)
        assert f.frame_id == d["frame_id"]


def test_our_latest_is_his_latest(real_log):
    """The reconstruction from ``tracks`` matches his ``targets`` exactly."""
    for d in records(real_log):
        ours = sorted((t.id, t.box, t.state, t.action) for t in parse_detail(d, 0.0).targets)
        his = sorted((t["id"], tuple(t["box"]), t["state"], t["action"]) for t in d["targets"])
        assert ours == his, f"frame {d['frame_id']}"


def test_the_fixtures_actually_contain_coasting_tracks(real_log):
    # Otherwise the test above could not tell dropping them from keeping them.
    assert any(t["misses"] > 0 for d in records(real_log) for t in d["tracks"])


def test_his_centroid_is_the_box_centre_so_it_is_not_the_ground_point(real_log):
    for d in records(real_log):
        for t in parse_detail(d, 0.0).targets:
            x0, y0, x1, y1 = t.box
            assert t.centroid == ((x0 + x1) / 2, (y0 + y1) / 2)
            assert t.ground_px == ((x0 + x1) / 2, float(y1))


def test_real_frames_project_with_a_1280x720_calibration(real_log):
    """The sim calibration is fictional, but it is at his real capture size."""
    robot = load_robot_config("config/sim_robot.yaml")
    calib = load_ground_calibration("config/sim_ground_homography.json", robot)
    proj = GroundProjector(calib, max_valid_range_m=robot.get("camera.fov_far_limit_m"))
    ts = TargetSet()
    n = 0
    for i, d in enumerate(records(real_log)):
        frame = parse_detail(d, 0.0)
        ts.update(frame, proj, Pose2D(), i / 30.0)
        n += len(select_targets(frame, states=(CONFIRM,), actions=(PICK,)))
    assert n > 0
    assert ts.tracks


def test_a_real_log_replays_whole(real_log):
    src = ReplayVisionSource(real_log)
    src.start()
    frames = src.poll()
    assert len(frames) == len(records(real_log))
    assert src.n_dropped == 0


def test_a_near_whole_frame_box_reaches_confirm():
    """Recorded, not endorsed: frame 4 of n640-c confirms a 1146x612 px box.

    Nav has no box-size gate. If one is ever added this documents why; if the
    detector stops doing it this test should be deleted, not loosened.
    """
    d = next(r for r in records(FIXTURES / "real_detail_n640.jsonl") if r["frame_id"] == 4)
    big = [t for t in parse_detail(d, 0.0).targets
           if t.box[2] - t.box[0] > 1000 and t.state == CONFIRM]
    assert big, json.dumps(d["targets"])
