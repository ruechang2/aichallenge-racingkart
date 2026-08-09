"""Which way the wheels point while backing out.

The sign is the whole trick. Yaw rate is v*tan(delta)/L, so reversing with the
steering angle that would fix the error going forwards makes it worse: the nose
swings further off the line. Reversing with the mirrored angle is what walks the
car back towards the racing line, and these tests pin that.

Pure geometry, no ROS:
    python3 -m pytest test/test_recovery_steering.py
"""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from stuck_recovery.recovery_logic import (  # noqa: E402
    mirrored_for_reverse, path_target, recovery_steer,
)

# A straight reference path along +x.
STRAIGHT = [(float(i), 0.0) for i in range(0, 60)]
K_LAT, K_PSI, MAX_STEER = 0.12, 0.8, 0.9


def steer_at(x, y, yaw, points=STRAIGHT, lookahead=6.0):
    lat, e_psi = path_target(points, x, y, yaw, lookahead)
    return recovery_steer(lat, e_psi, K_LAT, K_PSI, MAX_STEER)


# --- path projection ---

def test_on_the_line_and_aligned_needs_no_correction():
    assert abs(steer_at(10.0, 0.0, 0.0)) < 1e-6


def test_path_to_the_left_reads_as_positive_lat():
    lat, _ = path_target(STRAIGHT, 10.0, -3.0, 0.0, 6.0)
    assert lat > 0.0


def test_heading_error_is_measured_against_the_path_direction():
    _, e_psi = path_target(STRAIGHT, 10.0, 0.0, math.radians(30.0), 6.0)
    assert math.isclose(e_psi, math.radians(-30.0), abs_tol=1e-6)


def test_a_closed_loop_wraps_instead_of_running_out_of_path():
    # A 40 m square loop; a lookahead from the last point must wrap to the first.
    loop = [(float(i), 0.0) for i in range(0, 10)] + [(10.0, float(i)) for i in range(0, 10)]
    loop += [(float(10 - i), 10.0) for i in range(0, 10)] + [(0.0, float(10 - i)) for i in range(0, 10)]
    assert path_target(loop, 0.0, 1.0, math.pi / 2, 6.0) is not None


# --- the sign that matters ---

def test_pointing_at_the_wall_steers_back_towards_the_line_going_forward():
    # Car off to the right of the line (y = -3) and pointing further right.
    steer = steer_at(10.0, -3.0, math.radians(-40.0))
    assert steer > 0.0, "forward correction must be to the left, back to the line"


def test_reversing_mirrors_that_correction():
    forward = steer_at(10.0, -3.0, math.radians(-40.0))
    assert mirrored_for_reverse(forward) == -forward


def test_mirrored_steer_rotates_the_nose_towards_the_path_while_reversing():
    """Integrate the bicycle model backwards and check the heading error shrinks."""
    x, y, yaw = 10.0, -3.0, math.radians(-40.0)
    wheel_base, speed, dt = 1.087, -2.0, 0.05  # negative speed == reversing

    def heading_error(psi):
        return abs(path_target(STRAIGHT, x, y, psi, 6.0)[1])

    before = heading_error(yaw)
    for _ in range(20):  # 1 s of backing up
        delta = mirrored_for_reverse(steer_at(x, y, yaw))
        yaw += speed * math.tan(delta) / wheel_base * dt
        x += speed * math.cos(yaw) * dt
        y += speed * math.sin(yaw) * dt
    assert heading_error(yaw) < before


def test_using_the_unmirrored_steer_in_reverse_would_make_it_worse():
    """The control case for the test above -- this is the bug being avoided."""
    x, y, yaw = 10.0, -3.0, math.radians(-40.0)
    wheel_base, speed, dt = 1.087, -2.0, 0.05

    before = abs(path_target(STRAIGHT, x, y, yaw, 6.0)[1])
    for _ in range(20):
        delta = steer_at(x, y, yaw)  # NOT mirrored
        yaw += speed * math.tan(delta) / wheel_base * dt
        x += speed * math.cos(yaw) * dt
        y += speed * math.sin(yaw) * dt
    assert abs(path_target(STRAIGHT, x, y, yaw, 6.0)[1]) > before


def test_steer_is_clamped_to_full_lock():
    # Spun right around: the raw correction is far past the steering limit.
    steer = steer_at(10.0, -20.0, math.pi)
    assert abs(steer) <= MAX_STEER + 1e-9
