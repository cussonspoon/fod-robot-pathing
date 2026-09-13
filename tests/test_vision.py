"""The vision link, against the real fod-vision v0.3.0 contract.

Field names and semantics come from docs/vendor/fod-vision-v0.3.0-INTEGRATION.md
§5. This is still our parser tested against our reading of his spec -- it is not
integration -- but the spec is now his, published and measured, rather than one
we invented and hoped he would implement.
"""

from __future__ import annotations

import json
import math

import pytest

from fodnav.link.vision import (
    CAUTION, CONFIRM, IGNORE, PICK, REPORT,
    JsonlVisionLog, QueueVisionSource, ReplayVisionSource, Target, VisionFrame,
    iter_jsonl, parse_detail, select_targets,
)

# A detail() dict in his documented shape (INTEGRATION.md §5.2).
DETAIL = {
    "frame_id": 2, "age": 0.018, "blocked": True, "fps": 30.0,
    "stage_ms": {"capture": 17.4, "preprocess": 1.1, "infer": 14.7,
                 "postprocess": 0.1, "total": 33.3},
    "top_scores": {"bolt": 0.91, "nut": 0.0, "screw": 0.0, "washer": 0.0},
    "camera": {"zoom": 1.0, "rotate": 0, "conf": 0.25,
               "frame_size": [1280, 720], "imgsz": 640, "focus_m": None},
    "tracks": [
        {"id": 911, "state": CONFIRM, "action": PICK, "cls": "bolt", "conf": 0.87,
         "raw": 0.91, "hits": 7, "misses": 0,
         "box": [610, 430, 646, 470], "centroid": [628.0, 450.0], "in_zone": True},
        {"id": 912, "state": CAUTION, "action": PICK, "cls": "screw", "conf": 0.31,
         "raw": 0.28, "hits": 2, "misses": 0,
         "box": [300, 500, 320, 515], "centroid": [310.0, 507.5], "in_zone": True},
        {"id": 913, "state": CONFIRM, "action": PICK, "cls": "nut", "conf": 0.70,
         "raw": 0.70, "hits": 9, "misses": 3,
         "box": [100, 600, 120, 615], "centroid": [110.0, 607.5], "in_zone": False},
    ],
    "error": None,
}


def detail(**over):
    d = json.loads(json.dumps(DETAIL))
    d.update(over)
    return d


# -- parsing --------------------------------------------------------------


def test_his_documented_shape_parses():
    f = parse_detail(detail())
    assert f.frame_id == 2
    assert f.frame_size == (1280, 720)
    assert f.age_s == pytest.approx(0.018)
    assert f.blocked is True
    assert f.ok


def test_coasting_tracks_are_not_in_latest():
    # detail()["tracks"] includes tracks riding through missed frames; latest()
    # is the subset with misses == 0. Track 913 has missed 3 and must not appear.
    f = parse_detail(detail())
    assert [t.id for t in f.targets] == [911, 912]


def test_the_ground_point_is_the_bottom_edge_not_the_centroid():
    # His centroid is the box CENTRE. Projecting it puts every object further
    # away than it is, by more the taller it is -- the exact failure CLAUDE.md §3
    # names. The bottom edge is the contact point with the floor.
    f = parse_detail(detail())
    t = f.targets[0]
    assert t.box == (610, 430, 646, 470)
    assert t.ground_px == (628.0, 470.0)      # y1, the bottom edge
    assert t.centroid == (628.0, 450.0)       # 20 px higher up the image
    assert t.ground_px[1] > t.centroid[1]


def test_a_dead_camera_is_carried_as_an_error_not_as_an_empty_floor():
    # His rule: "no debris, keep patrolling is the one thing a broken camera
    # must never look like". detail() never raises, so the reason arrives on
    # the frame and the FSM stops on it.
    f = parse_detail(detail(error="RuntimeError('hailo device gone')", tracks=[]))
    assert not f.ok
    assert "hailo" in f.error
    assert f.targets == ()


def test_age_of_infinity_survives_the_round_trip():
    f = parse_detail(detail(age=None))
    assert math.isinf(f.age_s)


def test_an_empty_frame_is_a_valid_answer():
    # Empty is "the floor is clear", which is not the same as no frame at all.
    f = parse_detail(detail(tracks=[], blocked=False))
    assert f.ok and f.targets == () and not f.blocked


# -- filtering ------------------------------------------------------------


def test_filtering_is_on_state_and_action_never_on_class():
    # His class names flip between frames on one object and disappear entirely
    # when the single-class arena dataset lands. Code that branches on state and
    # action survives that; code that branches on cls does not.
    f = parse_detail(detail())
    kept = select_targets(f)
    assert [t.id for t in kept] == [911]          # 912 is only CAUTION
    assert all(t.state == CONFIRM for t in kept)


def test_a_class_rename_changes_nothing():
    renamed = detail()
    for tr in renamed["tracks"]:
        tr["cls"] = "metal_fastener"              # what FR-3 ships
    assert [t.id for t in select_targets(parse_detail(renamed))] == [911]


def test_caution_can_be_admitted_deliberately():
    f = parse_detail(detail())
    assert [t.id for t in select_targets(f, states=(CONFIRM, CAUTION))] == [911, 912]


def test_an_ignore_action_is_dropped():
    d = detail()
    d["tracks"][0]["action"] = IGNORE
    assert select_targets(parse_detail(d)) == ()


def test_report_is_never_expected_to_fire():
    # His FR-13 vs FR-3 note: a single-class detector cannot tell ferrous from
    # non-ferrous, so action is constant PICK. Nothing may depend on REPORT.
    f = parse_detail(detail())
    assert all(t.action == PICK for t in f.targets)


# -- sources --------------------------------------------------------------


def test_a_queue_source_hands_frames_over():
    s = QueueVisionSource()
    assert s.poll() == []
    s.offer(detail())
    (f,) = s.poll()
    assert f.frame_id == 2
    assert s.poll() == []                          # drained
    assert s.stats.frames == 1 and s.stats.targets_seen == 2


def test_error_frames_are_counted():
    s = QueueVisionSource()
    s.offer(detail(error="boom", tracks=[]))
    s.poll()
    assert s.stats.errors == 1 and s.stats.last_error == "boom"


# -- log and replay -------------------------------------------------------


def test_his_detail_is_the_log_format_verbatim(tmp_path):
    # Nav invents no schema of its own here. One shape, and he owns it, so the
    # two sides cannot drift apart over field names.
    path = tmp_path / "detections.jsonl"
    with JsonlVisionLog(path) as log:
        for i in range(4):
            log.write(detail(frame_id=i))
    records = list(iter_jsonl(path))
    assert [r["frame_id"] for r in records] == [0, 1, 2, 3]
    assert records[0]["camera"]["frame_size"] == [1280, 720]


def test_a_long_log_replays_whole_without_dropping_frames(tmp_path):
    # The live sources bound their queue so a stalled loop loses the oldest
    # frames rather than acting on stale ones. Replay must not: a recorded log
    # is evidence and dropping most of it silently is worse than useless.
    path = tmp_path / "long.jsonl"
    with JsonlVisionLog(path) as log:
        for i in range(500):
            log.write(detail(frame_id=i))
    src = ReplayVisionSource(path, speed=0.0)
    src.start()
    frames = src.poll()
    assert len(frames) == 500
    assert src.n_dropped == 0


def test_a_recorded_log_replays_through_the_same_interface(tmp_path):
    path = tmp_path / "detections.jsonl"
    with JsonlVisionLog(path) as log:
        for i in range(5):
            log.write(detail(frame_id=i))
    src = ReplayVisionSource(path, speed=0.0)
    src.start()
    frames = src.poll()
    assert [f.frame_id for f in frames] == [0, 1, 2, 3, 4]
    assert src.finished


def test_a_broken_log_line_is_skipped_not_fatal(tmp_path):
    path = tmp_path / "d.jsonl"
    path.write_text(json.dumps(detail()) + "\nthis was never JSON\n" + json.dumps(detail(frame_id=9)) + "\n")
    src = ReplayVisionSource(path, speed=0.0)
    src.start()
    assert [f.frame_id for f in src.poll()] == [2, 9]


def test_a_log_write_failure_does_not_stop_the_robot(tmp_path):
    log = JsonlVisionLog(tmp_path / "d.jsonl")
    log.close()
    log.write(detail())                            # writing to a closed file
    assert log.n_written == 0                      # counted as not written, no raise


# -- the library source, without a Pi --------------------------------------


def test_the_library_is_not_imported_until_start():
    # fodcv needs picamera2 and hailo_platform, which exist only on the Pi.
    # Constructing the source must not import it, or nothing in this repo would
    # be importable on a laptop (CLAUDE.md §2).
    import sys

    from fodnav.link.vision import LibraryVisionSource

    src = LibraryVisionSource(hef="nowhere/best.hef")
    assert "fodcv" not in sys.modules
    assert src.poll() == []                        # not started: no frames, no raise
    assert math.isinf(src.age_s)
