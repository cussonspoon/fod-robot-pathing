"""Association and hysteresis: many boxes per second, one thing to chase."""

from __future__ import annotations

import pytest

from fodnav.config import load_nav_config
from fodnav.frames import Pose2D
from fodnav.ground import GroundPoint
from fodnav.target import TargetTracker, TrackerParams

P = TrackerParams(acquire_conf=0.5, drop_conf=0.3, assoc_max_jump_m=0.2, min_hits=3, max_misses=5)


def obs(*points):
    return [(GroundPoint(x, y), c, "bolt") for x, y, c in points]


def feed(tr, points, n, t0=0.0, dt=1 / 30):
    for i in range(n):
        tr.update(obs(*points), t0 + i * dt)
    return tr


def test_the_shipped_parameters_load():
    p = TrackerParams.from_config(load_nav_config("config/nav.yaml"))
    assert p.drop_conf <= p.acquire_conf


def test_a_steady_detection_becomes_one_confirmed_track():
    tr = TargetTracker(P)
    feed(tr, [(0.8, 0.1, 0.9)], 5)
    assert len(tr.tracks) == 1
    assert tr.confirmed()[0].hits >= P.min_hits


def test_a_track_is_not_acted_on_until_it_has_been_seen_enough():
    # One frame is a flicker. min_hits is what stops the robot lurching at one.
    tr = TargetTracker(P)
    tr.update(obs((0.8, 0.1, 0.9)), 0.0)
    assert tr.best() is None
    feed(tr, [(0.8, 0.1, 0.9)], P.min_hits, t0=0.1)
    assert tr.best() is not None


def test_a_moving_target_stays_one_track():
    tr = TargetTracker(P)
    for i in range(20):
        tr.update(obs((1.2 - 0.02 * i, 0.1, 0.9)), i / 30)
    assert len(tr.tracks) == 1
    assert tr.best().x == pytest.approx(1.2 - 0.02 * 19)


def test_a_jump_beyond_the_gate_is_a_different_object():
    tr = TargetTracker(P)
    feed(tr, [(0.8, 0.0, 0.9)], 5)
    tr.update(obs((0.8 + 2 * P.assoc_max_jump_m, 0.0, 0.9)), 1.0)
    assert len(tr.tracks) == 2


def test_two_objects_stay_two_tracks():
    tr = TargetTracker(P)
    feed(tr, [(0.8, -0.3, 0.9), (1.2, 0.4, 0.8)], 5)
    assert len(tr.confirmed()) == 2


def test_confidence_hysteresis_acquires_high_and_holds_low():
    # One threshold would let a detector sitting near it flicker, and a
    # flickering target thrashes the FSM.
    tr = TargetTracker(P)
    tr.update(obs((0.8, 0.0, 0.45)), 0.0)
    assert tr.tracks == [], "below acquire_conf, no track is started"
    feed(tr, [(0.8, 0.0, 0.6)], 4, t0=0.1)
    assert tr.best() is not None
    feed(tr, [(0.8, 0.0, 0.35)], 4, t0=0.5)
    assert tr.best() is not None, "above drop_conf, it survives"
    tr.update(obs((0.8, 0.0, 0.2)), 1.0)
    assert tr.best() is None, "below drop_conf, it goes"


def test_a_track_survives_a_few_dropped_frames():
    tr = TargetTracker(P)
    feed(tr, [(0.8, 0.0, 0.9)], 5)
    for i in range(P.max_misses):
        tr.update([], 1.0 + i / 30)
    assert tr.best() is not None
    for i in range(P.max_misses + 1):
        tr.update([], 2.0 + i / 30)
    assert tr.best() is None


def test_best_is_the_nearest_confirmed_target():
    tr = TargetTracker(P)
    feed(tr, [(1.5, 0.0, 0.95), (0.7, 0.1, 0.55)], 5)
    # Nearest, not most confident: it is the cheapest to reach and the most
    # likely to still be there on arrival.
    assert tr.best().x == pytest.approx(0.7)


def test_best_respects_a_range_limit():
    tr = TargetTracker(P)
    feed(tr, [(1.5, 0.0, 0.9)], 5)
    assert tr.best(max_range_m=1.0) is None
    assert tr.best(max_range_m=2.0) is not None


def test_a_stale_estimate_is_aged_forward_on_odometry():
    # The loop runs at 50 Hz on a 30 Hz stream, so most ticks use a measurement
    # up to 33 ms old. Rotating it into the current frame is what t_capture is
    # for, and it is enough to move where the blind leg latches.
    tr = TargetTracker(P)
    at_measure = Pose2D(0.0, 0.0, 0.0)
    feed_pts = obs((1.0, 0.0, 0.9))
    for i in range(5):
        tr.update(feed_pts, i / 30, at_measure)
    track = tr.best()
    moved = Pose2D(0.1, 0.0, 0.0)  # the robot advanced 10 cm since
    predicted = track.predict_base(moved)
    assert predicted.x == pytest.approx(0.9)
    assert track.x == pytest.approx(1.0), "the stored measurement is not mutated"


def test_ageing_handles_rotation_too():
    import math

    tr = TargetTracker(P)
    for i in range(5):
        tr.update(obs((1.0, 0.0, 0.9)), i / 30, Pose2D(0, 0, 0))
    p = tr.best().predict_base(Pose2D(0.0, 0.0, math.pi / 2))
    assert (p.x, p.y) == pytest.approx((0.0, -1.0), abs=1e-9)


def test_reset_clears_everything():
    tr = TargetTracker(P)
    feed(tr, [(0.8, 0.0, 0.9)], 5)
    tr.reset()
    assert tr.tracks == [] and tr.best() is None
