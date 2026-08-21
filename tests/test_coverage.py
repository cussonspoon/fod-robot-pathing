"""Swept-cell bookkeeping: what the drum actually passed over."""

from __future__ import annotations

import math

import pytest

from fodnav.frames import Pose2D
from fodnav.planner.boustrophedon import Rect
from fodnav.planner.coverage import CoverageMap


def test_a_fresh_map_is_empty():
    cm = CoverageMap(Rect(0, 0, 1, 1), cell_m=0.1)
    assert cm.n_cells == 100
    assert cm.fraction_covered() == 0.0


def test_driving_across_marks_a_stripe():
    cm = CoverageMap(Rect(0, 0, 1, 1), cell_m=0.05)
    for i in range(101):
        cm.sweep(Pose2D(i / 100.0, 0.5, 0.0), drum_width_m=0.2)
    assert cm.covered(0.5, 0.5)
    assert cm.covered(0.5, 0.55)
    assert not cm.covered(0.5, 0.9)
    # A 20 cm drum over a 1 m square is about a fifth of it.
    assert 0.15 < cm.fraction_covered() < 0.30


def test_the_drum_is_marked_where_the_drum_is_not_where_the_robot_is():
    # The drum is behind the axle; marking the robot's own position would
    # report a strip as swept that the magnets never passed over.
    cm = CoverageMap(Rect(0, 0, 2, 1), cell_m=0.02)
    cm.sweep(Pose2D(1.0, 0.5, 0.0), drum_width_m=0.1, drum_offset_x_m=-0.3)
    assert cm.covered(0.7, 0.5)
    assert not cm.covered(1.0, 0.5)


def test_the_drum_lies_across_the_direction_of_travel():
    cm = CoverageMap(Rect(0, 0, 2, 2), cell_m=0.02)
    cm.sweep(Pose2D(1.0, 1.0, 0.0), drum_width_m=0.4)  # facing +x
    assert cm.covered(1.0, 1.15) and cm.covered(1.0, 0.85)
    assert not cm.covered(1.15, 1.0)

    cm2 = CoverageMap(Rect(0, 0, 2, 2), cell_m=0.02)
    cm2.sweep(Pose2D(1.0, 1.0, math.pi / 2), drum_width_m=0.4)  # facing +y
    assert cm2.covered(1.15, 1.0) and cm2.covered(0.85, 1.0)
    assert not cm2.covered(1.0, 1.15)


def test_a_jump_between_ticks_is_interpolated():
    # A stalled loop or a log replayed fast makes the pose jump. Recording the
    # skipped strip as unswept would be a lie about the run.
    jumpy = CoverageMap(Rect(0, 0, 2, 1), cell_m=0.05)
    jumpy.sweep(Pose2D(0.1, 0.5, 0.0), 0.2)
    jumpy.sweep(Pose2D(1.9, 0.5, 0.0), 0.2)
    assert jumpy.covered(1.0, 0.5), "the strip between the two poses is swept"


def test_sweeping_outside_the_arena_is_ignored_not_an_error():
    cm = CoverageMap(Rect(0, 0, 1, 1), cell_m=0.1)
    cm.sweep(Pose2D(5.0, 5.0, 0.0), 0.2)
    assert cm.fraction_covered() == 0.0


def test_uncovered_cells_locate_the_holes():
    cm = CoverageMap(Rect(0, 0, 1, 1), cell_m=0.1)
    for i in range(101):
        cm.sweep(Pose2D(i / 100.0, 0.5, 0.0), drum_width_m=0.2)
    holes = cm.uncovered_cells()
    assert holes
    assert all(not cm.covered(x, y) for x, y in holes)
    assert all(abs(y - 0.5) > 0.05 for _x, y in holes)


def test_the_ascii_art_puts_plus_y_at_the_top():
    cm = CoverageMap(Rect(0, 0, 1, 1), cell_m=0.1)
    cm.sweep(Pose2D(0.5, 0.95, 0.0), drum_width_m=0.2)
    art = cm.ascii_art(width=10).splitlines()
    assert "#" in art[0], "a sweep at high y should appear at the top of the picture"
    assert "#" not in art[-1]


def test_a_nonsense_cell_size_is_refused():
    with pytest.raises(ValueError):
        CoverageMap(Rect(0, 0, 1, 1), cell_m=0.0)


def test_the_summary_is_loggable():
    cm = CoverageMap(Rect(0, 0, 1, 1), cell_m=0.1)
    s = cm.summary()
    assert set(s) == {"cells", "covered", "fraction", "cell_m"}
