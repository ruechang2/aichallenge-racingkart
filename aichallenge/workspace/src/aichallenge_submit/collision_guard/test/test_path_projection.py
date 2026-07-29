"""Geometry tests for the curved-corridor projection.

These are pure math and need neither ROS nor a running node, so they are the
cheapest place to catch a sign error in the arc projection — which would either
make the guard brake for karts beside us (deadlocking every overtake) or, worse,
ignore a kart genuinely in our path.

Run with:  python3 -m pytest test/test_path_projection.py
"""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from collision_guard.geometry import project_onto_path  # noqa: E402


def test_straight_path_is_plain_forward_and_lateral():
    along, offset = project_onto_path(10.0, 2.0, 0.0)
    assert along == 10.0
    assert offset == 2.0


def test_point_behind_reports_negative_along():
    along, _ = project_onto_path(-5.0, 0.0, 0.0)
    assert along < 0.0
    along, _ = project_onto_path(-5.0, 0.0, 0.1)
    assert along < 0.0


def test_point_exactly_on_left_arc_has_zero_offset():
    # kappa = 0.1 -> R = 10 m, centre at (0, 10). A quarter turn puts the arc at
    # (10, 10), a quarter of the circumference (15.71 m) along.
    along, offset = project_onto_path(10.0, 10.0, 0.1)
    assert offset == 0.0
    assert math.isclose(along, 10.0 * math.pi / 2, rel_tol=1e-9)


def test_right_turn_mirrors_left_turn():
    left = project_onto_path(10.0, 10.0, 0.1)
    right = project_onto_path(10.0, -10.0, -0.1)
    assert math.isclose(left[0], right[0], rel_tol=1e-9)
    assert math.isclose(left[1], right[1], rel_tol=1e-9)


def test_kart_straight_ahead_while_turning_is_off_the_arc():
    """The case that used to deadlock the car.

    Turning left at R = 10 m with a kart 10 m dead ahead: it is *not* on our
    path, and the guard must see a large offset so it does not brake.
    """
    _along, offset = project_onto_path(10.0, 0.0, 0.1)
    assert offset > 4.0


def test_kart_on_the_inside_of_the_corner_is_off_the_arc():
    # Kart 3 m to the left at 4 m ahead while we turn left at R = 10 m: it cuts
    # inside our arc, so we sweep around the outside of it and it is not in path.
    _along, offset = project_onto_path(4.0, 3.0, 0.1)
    assert offset > 1.8


def test_a_point_that_happens_to_lie_on_the_arc_reads_as_in_path():
    # (6, 2) is exactly on the R = 10 m circle centred at (0, 10) — a 6-8-10
    # triangle. Being off to the side is not by itself enough to be ignored;
    # what matters is distance from the arc, and here it is zero.
    _along, offset = project_onto_path(6.0, 2.0, 0.1)
    assert offset == 0.0


def test_kart_dead_ahead_on_a_straight_is_in_path():
    _along, offset = project_onto_path(8.0, 0.3, 0.0)
    assert offset < 1.8


def test_tight_arc_curls_back_onto_a_kart_beside_us():
    """Why a swept-angle limit is needed, not just a distance limit.

    Observed in the sim: steering hard (R = 1.18 m) with a kart 1.84 m ahead and
    3.21 m to the left. Extrapolating that arc sweeps 138 deg and puts the kart
    'in path' at 2.8 m, which triggered a false emergency stop. The projection
    itself is correct — the caller must reject sweeps this large.
    """
    along, offset = project_onto_path(1.84, 3.21, 1.0 / 1.18)
    assert offset < 1.8          # reads as in-path...
    sweep = along * (1.0 / 1.18)
    assert sweep > 1.57          # ...only by curling most of the way around


def test_offset_grows_smoothly_as_we_steer_away():
    """Steering harder must monotonically push a dead-ahead kart off our arc."""
    offsets = [project_onto_path(8.0, 0.0, k)[1] for k in (0.0, 0.02, 0.05, 0.1)]
    assert offsets == sorted(offsets)
    assert offsets[0] == 0.0
