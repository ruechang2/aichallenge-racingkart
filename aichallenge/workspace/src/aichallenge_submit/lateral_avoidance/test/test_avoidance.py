"""Which way round, how hard, and when to refuse.

The expensive mistakes here are a sign error (steering into the kart instead of
around it) and steering off the track to avoid a kart, which trades a contact
penalty for a beached session. Both are pinned below.

    cd aichallenge/workspace/src/aichallenge_submit/lateral_avoidance
    python3 -m pytest test/ -q
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from lateral_avoidance.avoidance import (  # noqa: E402
    blend, curvature_for_shift, pass_side, steering_bias,
)


CLEAR = 1.6      # [m] lateral gap we want when passing
ROOM = 1.6       # [m] how far off the racing line we are willing to go


# --- which side ---

def test_a_kart_dead_ahead_is_passed_on_the_shortest_side():
    shift = pass_side(kart_lat=0.0, ego_offset=0.0, pass_clearance=CLEAR, max_offset=ROOM)
    assert abs(shift) == pytest.approx(CLEAR)


def test_a_kart_slightly_left_is_passed_on_the_right():
    shift = pass_side(kart_lat=0.3, ego_offset=0.0, pass_clearance=CLEAR, max_offset=ROOM)
    assert shift < 0.0, "kart on the left means go right"
    assert shift == pytest.approx(-(CLEAR - 0.3))


def test_a_kart_slightly_right_is_passed_on_the_left():
    shift = pass_side(kart_lat=-0.3, ego_offset=0.0, pass_clearance=CLEAR, max_offset=ROOM)
    assert shift > 0.0, "kart on the right means go left"
    assert shift == pytest.approx(CLEAR - 0.3)


def test_a_kart_already_wide_enough_gets_no_correction():
    assert pass_side(kart_lat=2.0, ego_offset=0.0,
                     pass_clearance=CLEAR, max_offset=ROOM) == 0.0
    assert pass_side(kart_lat=-2.0, ego_offset=0.0,
                     pass_clearance=CLEAR, max_offset=ROOM) == 0.0


# --- refusing to leave the track ---

def test_no_room_either_side_means_do_not_avoid():
    """Following a kart costs seconds; driving off the surface costs the session."""
    assert pass_side(kart_lat=0.0, ego_offset=0.0,
                     pass_clearance=CLEAR, max_offset=0.5) is None


def test_the_cheap_side_is_refused_when_it_runs_out_of_track():
    # Kart just left of us, so the cheap move is right — but we are already 1.4 m
    # right and only 1.6 m is allowed, so right is unavailable and left is taken.
    shift = pass_side(kart_lat=0.2, ego_offset=-1.4, pass_clearance=CLEAR, max_offset=ROOM)
    assert shift is not None and shift > 0.0, "must switch to the side with room"
    assert abs(-1.4 + shift) <= ROOM


def test_a_side_is_only_available_if_the_whole_shift_fits():
    shift = pass_side(kart_lat=0.0, ego_offset=1.0, pass_clearance=CLEAR, max_offset=ROOM)
    assert shift == pytest.approx(-CLEAR), "left would end at 2.6 m off line; go right"
    assert abs(1.0 + shift) <= ROOM


# --- how hard ---

def test_the_bias_pushes_towards_the_chosen_side():
    right = steering_bias(-1.3, preview=12.0, gain=1.639, max_curvature=0.3)
    left = steering_bias(1.3, preview=12.0, gain=1.639, max_curvature=0.3)
    assert right < 0.0 < left
    assert right == pytest.approx(-left)


def test_a_longer_preview_needs_less_curvature():
    near = abs(curvature_for_shift(1.3, 8.0))
    far = abs(curvature_for_shift(1.3, 16.0))
    assert far < near


def test_curvature_is_clamped_in_curvature_space_not_after_the_gain():
    """Clamping the scaled command would make the vehicle limit depend on the gain."""
    gain, limit = 1.639, 0.3
    bias = steering_bias(5.0, preview=3.0, gain=gain, max_curvature=limit)
    assert bias == pytest.approx(limit * gain)


def test_no_shift_means_no_bias():
    assert steering_bias(0.0, preview=12.0, gain=1.639, max_curvature=0.3) == 0.0


def test_a_zero_preview_cannot_divide_by_zero():
    assert curvature_for_shift(1.0, 0.0) == 0.0


# --- rate limiting ---

def test_the_bias_ramps_rather_than_stepping():
    assert blend(0.0, 1.0, 0.25) == pytest.approx(0.25)
    assert blend(0.0, -1.0, 0.25) == pytest.approx(-0.25)


def test_it_settles_exactly_on_the_target():
    assert blend(0.9, 1.0, 0.25) == pytest.approx(1.0)


def test_releasing_is_rate_limited_too():
    """A kart flickering in and out of the corridor must not shake the steering."""
    assert blend(1.0, 0.0, 0.25) == pytest.approx(0.75)
