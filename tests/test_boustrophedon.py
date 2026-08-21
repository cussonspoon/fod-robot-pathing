"""The coverage planner. Pure, so there is no excuse for testing it lightly.

The four properties named in CLAUDE.md section 9 are the first four groups
below, checked across a spread of rectangles, swaths, overlaps, corners and
orientations rather than on one example.
"""

from __future__ import annotations

import math

import pytest

from fodnav.config import load_nav_config, load_robot_config
from fodnav.planner.boustrophedon import (
    CORNERS,
    ORIENTATIONS,
    Rect,
    coverage_gaps,
    path_length,
    plan,
    row_count,
    row_spacing,
    swath_width_from_config,
)

RECTS = [
    Rect(0, 0, 3, 3),          # the seeded grid
    Rect(0, 0, 4, 2),          # wide
    Rect(0, 0, 2, 5),          # tall
    Rect(-1, -2, 1.5, 0.5),    # negative coordinates
    Rect(0, 0, 1.0, 0.35),     # narrower than one swath in y
    Rect(0, 0, 0.4, 0.4),      # smaller than one swath in both
]
SWATHS = [0.18, 0.22, 0.5, 0.64]
OVERLAPS = [0.0, 0.15, 0.5]


def all_plans():
    for rect in RECTS:
        for w in SWATHS:
            for ov in OVERLAPS:
                for corner in CORNERS:
                    for orient in ORIENTATIONS:
                        yield rect, w, ov, corner, orient


# -- property 1: everything gets swept -----------------------------------


@pytest.mark.parametrize("rect", RECTS)
@pytest.mark.parametrize("w", SWATHS)
@pytest.mark.parametrize("ov", OVERLAPS)
def test_every_point_is_within_half_a_spacing_of_the_path(rect, w, ov):
    wps = plan(rect, w, ov)
    gaps = coverage_gaps(rect, wps, row_spacing(w, ov), samples=40)
    assert gaps == [], f"{len(gaps)} uncovered points, e.g. {gaps[:3]}"


@pytest.mark.parametrize("corner", CORNERS)
@pytest.mark.parametrize("orient", ORIENTATIONS)
def test_coverage_holds_for_every_corner_and_orientation(corner, orient):
    rect, w, ov = Rect(0, 0, 3, 2), 0.22, 0.15
    wps = plan(rect, w, ov, start_corner=corner, orientation=orient)
    assert coverage_gaps(rect, wps, row_spacing(w, ov), samples=40) == []


def test_a_gap_is_detected_when_there_is_one():
    # The coverage checker has to be able to fail, or the tests above prove
    # nothing about the planner.
    rect = Rect(0, 0, 3, 3)
    sparse = plan(rect, 0.22, 0.15)[:4]
    assert coverage_gaps(rect, sparse, row_spacing(0.22, 0.15), samples=20) != []


# -- property 2: nothing leaves the rectangle ----------------------------


def test_no_waypoint_lies_outside_the_rectangle():
    # The obvious implementation steps rows from one edge and puts the last one
    # just past the far edge whenever the span is not an exact multiple of the
    # spacing, which is almost always.
    for rect, w, ov, corner, orient in all_plans():
        for x, y in plan(rect, w, ov, corner, orient):
            assert rect.contains(x, y), f"{(x, y)} outside {rect} for swath {w}, overlap {ov}"


# -- property 3: the count matches the closed form -----------------------


@pytest.mark.parametrize("rect", RECTS)
@pytest.mark.parametrize("w", SWATHS)
@pytest.mark.parametrize("ov", OVERLAPS)
def test_the_waypoint_count_matches_the_closed_form(rect, w, ov):
    spacing = row_spacing(w, ov)
    span = rect.height if rect.long_axis == "x" else rect.width
    assert len(plan(rect, w, ov)) == 2 * row_count(span, spacing)


@pytest.mark.parametrize(
    "span, spacing, expect",
    [(3.0, 1.0, 3), (3.0, 0.99, 4), (1.0, 2.0, 1), (0.0001, 1.0, 1), (2.6, 0.153, 17)],
)
def test_row_count_is_the_ceiling_of_the_ratio(span, spacing, expect):
    assert row_count(span, spacing) == expect


def test_an_exact_multiple_does_not_add_a_spurious_row():
    # Float trouble: 3.0 / 0.5 is 6.000000000000001 on some paths, and a naive
    # ceil turns that into seven rows and a wasted pass.
    assert row_count(3.0, 0.5) == 6
    assert row_count(2.6, 0.65) == 4


def test_row_spacing_applies_the_overlap():
    assert row_spacing(0.22, 0.0) == pytest.approx(0.22)
    assert row_spacing(0.22, 0.15) == pytest.approx(0.187)
    assert row_spacing(0.5, 0.5) == pytest.approx(0.25)


# -- property 4: reversing the start corner mirrors the path -------------


def assert_points_close(got, want):
    """Element-wise, because pytest.approx over a list of tuples is unreliable."""
    assert len(got) == len(want), f"{len(got)} points vs {len(want)}"
    for (gx, gy), (wx, wy) in zip(got, want):
        assert (gx, gy) == pytest.approx((wx, wy), abs=1e-9)


def test_starting_from_the_other_end_mirrors_in_x():
    rect = Rect(0, 0, 3, 2)
    sw = plan(rect, 0.3, 0.1, "sw")
    se = plan(rect, 0.3, 0.1, "se")
    mirrored = [(rect.x_min + rect.x_max - x, y) for x, y in sw]
    assert_points_close(se, mirrored)


def test_starting_from_the_other_side_mirrors_in_y():
    rect = Rect(0, 0, 3, 2)
    sw = plan(rect, 0.3, 0.1, "sw")
    nw = plan(rect, 0.3, 0.1, "nw")
    mirrored = [(x, rect.y_min + rect.y_max - y) for x, y in sw]
    assert_points_close(nw, mirrored)


def test_the_opposite_corner_mirrors_in_both():
    rect = Rect(0, 0, 3, 2)
    sw = plan(rect, 0.3, 0.1, "sw")
    ne = plan(rect, 0.3, 0.1, "ne")
    mirrored = [(rect.x_min + rect.x_max - x, rect.y_min + rect.y_max - y) for x, y in sw]
    assert_points_close(ne, mirrored)


# -- shape of the path ----------------------------------------------------


def test_it_starts_at_the_corner_it_was_told_to():
    rect = Rect(0, 0, 3, 2)
    assert plan(rect, 0.5, 0.0, "sw")[0][0] == pytest.approx(0.0)
    assert plan(rect, 0.5, 0.0, "se")[0][0] == pytest.approx(3.0)
    assert plan(rect, 0.5, 0.0, "nw")[0][1] > 1.0
    assert plan(rect, 0.5, 0.0, "sw")[0][1] < 1.0


def test_rows_run_along_the_long_axis_by_default():
    # Turns are the expensive part, so fewer, longer rows.
    wide = plan(Rect(0, 0, 4, 1), 0.25)
    tall = plan(Rect(0, 0, 1, 4), 0.25)
    assert wide[0][1] == wide[1][1], "a wide arena gets rows along x"
    assert tall[0][0] == tall[1][0], "a tall arena gets rows along y"
    assert len(wide) == len(tall)


def test_orientation_can_be_forced_against_the_long_axis():
    short = plan(Rect(0, 0, 4, 1), 0.25, orientation="short")
    assert short[0][0] == short[1][0]
    assert len(short) > len(plan(Rect(0, 0, 4, 1), 0.25, orientation="long"))


def test_running_rows_the_short_way_costs_more_turns():
    rect = Rect(0, 0, 4, 1)
    assert len(plan(rect, 0.25, orientation="short")) > len(
        plan(rect, 0.25, orientation="long")
    )


def test_the_rows_alternate_direction():
    # Boustrophedon: as the ox turns. Rows that all ran the same way would need
    # a full return leg between each.
    wps = plan(Rect(0, 0, 3, 2), 0.5, 0.0, "sw")
    directions = [math.copysign(1, b[0] - a[0]) for a, b in zip(wps[::2], wps[1::2])]
    assert all(a != b for a, b in zip(directions, directions[1:]))


def test_only_row_endpoints_are_emitted():
    # The controller handles the in-place turns; the planner does not emit
    # points along a straight row.
    wps = plan(Rect(0, 0, 3, 2), 0.5, 0.0)
    assert len(wps) == 2 * row_count(2.0, 0.5)


def test_a_narrow_strip_gets_one_row_down_the_middle():
    wps = plan(Rect(0, 0, 2, 0.3), 0.5)
    assert len(wps) == 2
    assert wps[0][1] == pytest.approx(0.15)


def test_path_length_is_the_driven_distance():
    wps = plan(Rect(0, 0, 3, 1), 0.5, 0.0)
    # 2 rows of 3 m plus one 0.5 m connector.
    assert path_length(wps) == pytest.approx(2 * 3.0 + 0.5)


# -- the rectangle itself -------------------------------------------------


def test_the_arena_is_inset_by_the_planner_margin():
    # The robot's axle midpoint cannot drive to the physical wall; the margin
    # is half the footprint plus a clearance, and it is measured.
    inner = Rect(0, 0, 3, 3).inset(0.2)
    assert (inner.x_min, inner.y_min, inner.x_max, inner.y_max) == (0.2, 0.2, 2.8, 2.8)


def test_a_margin_that_swallows_the_arena_is_refused():
    with pytest.raises(ValueError, match="degenerate"):
        Rect(0, 0, 1, 1).inset(0.6)


@pytest.mark.parametrize("bad", [(0, 0, 0, 1), (0, 0, 1, 0), (1, 0, 0, 1)])
def test_a_degenerate_rectangle_is_refused(bad):
    with pytest.raises(ValueError, match="degenerate"):
        Rect(*bad)


def test_the_long_axis_is_the_long_one():
    assert Rect(0, 0, 4, 1).long_axis == "x"
    assert Rect(0, 0, 1, 4).long_axis == "y"
    assert Rect(0, 0, 2, 2).long_axis == "x"  # square: pick one, deterministically


# -- bad arguments --------------------------------------------------------


@pytest.mark.parametrize("w, ov", [(0.0, 0.0), (-0.2, 0.0), (0.2, 1.0), (0.2, -0.1), (0.2, 1.5)])
def test_impossible_swath_parameters_are_refused(w, ov):
    with pytest.raises(ValueError):
        plan(Rect(0, 0, 3, 3), w, ov)


def test_an_unknown_corner_or_orientation_is_refused():
    with pytest.raises(ValueError, match="start_corner"):
        plan(Rect(0, 0, 3, 3), 0.2, 0.0, "north")
    with pytest.raises(ValueError, match="orientation"):
        plan(Rect(0, 0, 3, 3), 0.2, 0.0, "sw", "diagonal")


# -- which swath ----------------------------------------------------------


def test_the_swath_source_is_resolved_and_reported():
    # CLAUDE.md §9: the two candidates are not the same number and the choice
    # is unresolved, so a run has to record which one it used.
    robot = load_robot_config("config/sim_robot.yaml")
    nav = load_nav_config("config/nav.yaml")
    width, source = swath_width_from_config(robot, nav)
    assert source == nav.get("planner.swath_source")
    assert width == robot.get("drum.width_m")


def test_the_two_candidate_swaths_differ_enough_to_matter():
    robot = load_robot_config("config/sim_robot.yaml")
    drum = robot.get("drum.width_m")
    camera = robot.get("camera.fov_width_at_lookahead_m")
    rect = Rect(0, 0, 3, 3)
    rows_drum = row_count(3.0, row_spacing(drum, 0.15))
    rows_camera = row_count(3.0, row_spacing(camera, 0.15))
    assert rows_drum > 2 * rows_camera, (
        "the collection and detection readings of 'coverage' give sweeps that "
        "differ by more than a factor of two -- which is why the run log has to "
        "say which was used"
    )
