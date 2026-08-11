"""The corridor must contain the car, or the MPC has no way back.

The QP pins its initial state to the measured ``e_y`` and constrains every later
step to the corridor. If the car sits outside that corridor the two cannot both
hold and the problem is infeasible — and relaxing the safety margin does not
rescue it, because relaxation bottoms out at the static track bounds and the car
can be past them. Measured symptom: stranded 2.0-2.3 m off the racing line at
full steering lock, motionless, MPC emitting a permanent stop.

Pure arithmetic on two arrays, so it runs without ROS or a map:

    python3 -m pytest test/test_corridor_admits_current_offset.py
"""

import numpy as np
import pytest

from multi_purpose_mpc_ros.core.reference_path import ReferencePath


admit = ReferencePath.admit_current_offset
SLACK = ReferencePath.RETURN_SLACK


def corridor(n=10, ub=0.85, lb=-0.85):
    return np.full(n, float(ub)), np.full(n, float(lb))


def test_a_car_inside_the_corridor_is_left_completely_alone():
    ub, lb = corridor()
    out_ub, out_lb = admit(ub.copy(), lb.copy(), e_y=0.3)
    assert np.array_equal(out_ub, ub)
    assert np.array_equal(out_lb, lb)


def test_a_car_outside_on_the_upper_side_gets_a_corridor_that_contains_it():
    ub, lb = corridor()
    out_ub, out_lb = admit(ub, lb, e_y=2.2)
    assert out_ub[0] >= 2.2, "the first constrained step must admit where the car is"
    assert out_ub[0] == pytest.approx(2.2 + SLACK)


def test_the_same_holds_on_the_lower_side():
    ub, lb = corridor()
    out_ub, out_lb = admit(ub, lb, e_y=-2.2)
    assert out_lb[0] <= -2.2
    assert out_lb[0] == pytest.approx(-2.2 - SLACK)


def test_only_the_violated_side_is_opened():
    """Opening both would invite the solver to cut across to the wrong side."""
    ub, lb = corridor()
    out_ub, out_lb = admit(ub, lb, e_y=2.2)
    assert np.array_equal(out_lb, np.full(10, -0.85))


def test_the_opening_tapers_across_the_whole_horizon_and_ends_nominal():
    """A short funnel is as infeasible as no funnel.

    Measured: corridor [-4.15, 1.90] with the car at e_y 2.53 and nearly stopped,
    opened over 5 steps, still would not solve — coming back 0.8 m within five
    waypoints is more than the steering rate can do at that speed. Spread over
    the horizon the same return asks for a few centimetres per step.
    """
    ub, lb = corridor(n=20)
    out_ub, _ = admit(ub, lb, e_y=2.2)
    assert all(a >= b for a, b in zip(out_ub, out_ub[1:])), "must shrink each step"
    assert out_ub[-1] == pytest.approx(0.85), "nominal again by the end"
    assert out_ub[0] - out_ub[1] < 0.1, "and gently: centimetres per waypoint"


def test_the_corridor_never_ends_up_inverted():
    """ub < lb makes OSQP infeasible and takes the whole node down with it."""
    ub, lb = corridor(ub=0.05, lb=-0.05)
    out_ub, out_lb = admit(ub, lb, e_y=3.0)
    assert np.all(out_ub >= out_lb)


def test_a_wildly_displaced_car_is_still_admitted():
    # After a spin or a shove there is no bound on how far off the line it is.
    ub, lb = corridor()
    out_ub, _ = admit(ub, lb, e_y=12.0)
    assert out_ub[0] >= 12.0


def test_nothing_blows_up_without_an_e_y_or_a_corridor():
    ub, lb = corridor()
    assert admit(ub, lb, e_y=None)[0] is ub
    empty_ub, empty_lb = np.array([]), np.array([])
    assert len(admit(empty_ub, empty_lb, e_y=2.0)[0]) == 0


def test_a_corridor_shorter_than_the_taper_is_handled():
    ub, lb = corridor(n=2)
    out_ub, _ = admit(ub, lb, e_y=2.2)
    assert len(out_ub) == 2
    assert out_ub[0] >= 2.2
