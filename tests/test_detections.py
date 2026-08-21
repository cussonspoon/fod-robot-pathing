"""The vision link. The schema is a request, not an agreement -- test it strictly."""

from __future__ import annotations

import json
import math
import time

import pytest

from fodnav.link.detections import (
    SCHEMA_VERSION,
    Detection,
    DetectionSchemaError,
    JsonlDetectionLog,
    QueueDetectionSource,
    ReplayDetectionSource,
    parse_message,
    select_targets,
)

# The example printed in CLAUDE.md section 8, verbatim. If this stops parsing,
# either the document or this module moved without the other.
CLAUDE_MD_EXAMPLE = json.loads(
    """
{
  "schema": 1,
  "t_capture": 1756000000.123,
  "t_publish": 1756000000.156,
  "frame_id": 4821,
  "frame_size": [2304, 1296],
  "dets": [
    {"cls": "bolt", "conf": 0.87, "bbox": [1102, 812, 61, 44]}
  ]
}
"""
)


def msg(**over):
    m = dict(CLAUDE_MD_EXAMPLE)
    m.update(over)
    return json.dumps(m)


# -- parsing --------------------------------------------------------------


def test_the_documented_example_parses():
    f = parse_message(msg())
    assert f.frame_id == 4821
    assert f.frame_size == (2304, 1296)
    assert f.t_capture == 1756000000.123
    assert len(f.dets) == 1
    d = f.dets[0]
    assert (d.cls, d.conf, d.bbox) == ("bolt", 0.87, (1102.0, 812.0, 61.0, 44.0))


def test_bytes_and_str_and_dict_all_parse():
    for payload in (msg(), msg().encode("utf-8"), CLAUDE_MD_EXAMPLE):
        assert parse_message(payload).frame_id == 4821


def test_the_ground_point_is_the_bottom_centre_not_the_centroid():
    d = Detection(cls="bolt", conf=0.9, x=100.0, y=200.0, w=60.0, h=40.0)
    assert d.ground_px == (130.0, 240.0)  # bottom edge: where it touches the floor
    assert d.centre_px == (130.0, 220.0)  # for drawing only


def test_an_empty_scene_is_a_valid_message():
    # And it is NOT the same as no message. Empty means the floor is clear;
    # silence means vision is dead. Conflating them is how a robot keeps
    # driving after the camera process dies.
    f = parse_message(msg(dets=[]))
    assert f.dets == ()


def test_unknown_fields_are_tolerated():
    # The real publisher is someone else's code and may grow a field. Growing
    # one must not take the robot down.
    f = parse_message(msg(hailo_ms=17.8, model="yolov8n"))
    assert f.frame_id == 4821


def test_a_missing_t_publish_falls_back_to_t_capture():
    m = dict(CLAUDE_MD_EXAMPLE)
    del m["t_publish"]
    assert parse_message(json.dumps(m)).pipeline_latency_s() == 0.0


@pytest.mark.parametrize(
    "payload, match",
    [
        ("not json at all", "not JSON"),
        ("[1, 2, 3]", "must be a JSON object"),
        (json.dumps({"schema": 2, "t_capture": 1.0, "frame_size": [1, 1]}), "schema version"),
        (json.dumps({"t_capture": 1.0, "frame_size": [1, 1]}), "schema version"),
    ],
)
def test_malformed_payloads_are_rejected(payload, match):
    with pytest.raises(DetectionSchemaError, match=match):
        parse_message(payload)


def test_a_missing_capture_timestamp_is_rejected():
    m = dict(CLAUDE_MD_EXAMPLE)
    del m["t_capture"]
    with pytest.raises(DetectionSchemaError, match="t_capture"):
        parse_message(json.dumps(m))


@pytest.mark.parametrize("size", [None, [2304], [2304, 1296, 3], ["2304", "1296"], [0, 1296], [-1, 5]])
def test_a_bad_frame_size_is_rejected(size):
    # frame_size is the only thing that makes "is this the resolution the
    # homography was calibrated at" checkable. Without it every projection is
    # silently wrong, so a message without a usable one is not usable.
    m = dict(CLAUDE_MD_EXAMPLE)
    if size is None:
        del m["frame_size"]
    else:
        m["frame_size"] = size
    with pytest.raises(DetectionSchemaError, match="frame_size"):
        parse_message(json.dumps(m))


@pytest.mark.parametrize(
    "bbox", [[1, 2, 3], [1, 2, 3, 4, 5], [1, 2, 0, 44], [1, 2, 61, -1], "1,2,3,4"]
)
def test_a_bad_bbox_is_rejected(bbox):
    with pytest.raises(DetectionSchemaError, match="bbox|dets"):
        parse_message(msg(dets=[{"cls": "bolt", "conf": 0.9, "bbox": bbox}]))


def test_a_non_string_class_is_rejected():
    with pytest.raises(DetectionSchemaError, match="cls"):
        parse_message(msg(dets=[{"cls": 3, "conf": 0.9, "bbox": [1, 2, 3, 4]}]))


# -- the nav-side class rules (CLAUDE.md section 8) -----------------------


TARGETS = ("nail", "screw", "bolt")


@pytest.mark.parametrize("cls", TARGETS)
def test_all_three_fastener_labels_are_one_target_class(cls):
    # The CV repo reliably finds screws and reliably calls them bolts. Nav must
    # not care which of the three came back.
    f = parse_message(msg(dets=[{"cls": cls, "conf": 0.9, "bbox": [1, 2, 61, 44]}]))
    assert len(select_targets(f, TARGETS, ("unknown",), 0.5)) == 1


def test_unknown_is_dropped_even_at_high_confidence():
    f = parse_message(msg(dets=[{"cls": "unknown", "conf": 0.99, "bbox": [1, 2, 61, 44]}]))
    assert select_targets(f, TARGETS, ("unknown",), 0.5) == ()


def test_low_confidence_boxes_are_dropped():
    f = parse_message(msg(dets=[{"cls": "bolt", "conf": 0.2, "bbox": [1, 2, 61, 44]}]))
    assert select_targets(f, TARGETS, ("unknown",), 0.5) == ()


def test_class_matching_is_case_insensitive():
    f = parse_message(msg(dets=[{"cls": "Bolt", "conf": 0.9, "bbox": [1, 2, 61, 44]}]))
    assert len(select_targets(f, TARGETS, ("unknown",), 0.5)) == 1


# -- sources --------------------------------------------------------------


def test_a_queue_source_hands_frames_to_the_caller():
    s = QueueDetectionSource()
    assert s.poll() == []
    assert s.seconds_since_last() == math.inf
    s.offer(msg())
    frames = s.poll()
    assert len(frames) == 1 and frames[0].frame_id == 4821
    assert s.poll() == []  # drained
    assert s.seconds_since_last() < 1.0


def test_malformed_messages_are_counted_not_raised():
    # A parse error must never propagate into the control loop. The vision
    # timeout is what stops the robot if they all stop conforming.
    s = QueueDetectionSource()
    s.offer("garbage")
    s.offer(msg())
    frames = s.poll()
    assert len(frames) == 1
    assert s.stats.malformed == 1 and s.stats.parsed == 1 and s.stats.received == 2


def test_a_malformed_message_still_proves_vision_is_alive():
    s = QueueDetectionSource()
    s.offer("garbage")
    assert s.seconds_since_last() < 1.0


def test_the_queue_is_bounded_and_drops_are_counted():
    s = QueueDetectionSource(maxlen=4)
    for i in range(10):
        s.offer(msg(frame_id=i))
    frames = s.poll()
    assert len(frames) == 4
    assert s.stats.dropped == 6
    # The survivors are the newest: acting on a stale box is worse than not acting.
    assert [f.frame_id for f in frames] == [6, 7, 8, 9]


# -- log and replay -------------------------------------------------------


def test_a_log_replays_through_the_same_interface(tmp_path):
    path = tmp_path / "dets.jsonl"
    with JsonlDetectionLog(path) as log:
        for i in range(5):
            log.write(msg(frame_id=i))
        log.write("this line was never valid JSON")
    src = ReplayDetectionSource(path, speed=0.0, rebase_capture_time=False)
    src.start()
    frames = src.poll()
    assert [f.frame_id for f in frames] == [0, 1, 2, 3, 4]
    assert src.stats.malformed == 1  # the junk line was kept, and is still junk
    assert src.finished


def test_replay_can_rebase_historical_timestamps_onto_now(tmp_path):
    path = tmp_path / "dets.jsonl"
    with JsonlDetectionLog(path) as log:
        log.write(msg())  # t_capture is from 2025
    src = ReplayDetectionSource(path, speed=0.0, rebase_capture_time=True)
    src.start()
    (frame,) = src.poll()
    assert abs(frame.age_s()) < 1.0, "a replayed frame must not look years stale"


def test_replay_respects_recorded_timing(tmp_path):
    path = tmp_path / "dets.jsonl"
    log = JsonlDetectionLog(path)
    log.write(msg(frame_id=0))
    time.sleep(0.05)
    log.write(msg(frame_id=1))
    log.close()
    src = ReplayDetectionSource(path, speed=1.0)
    src.start()
    first = src.poll()
    assert [f.frame_id for f in first] == [0], "the second frame is not due yet"
    time.sleep(0.08)
    assert [f.frame_id for f in src.poll()] == [1]
