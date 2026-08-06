"""The emergency rule: can we still stop, or not?

Judging an emergency by raw clearance alone made the car emergency-brake *while
already stopped*, so it could never pull off the starting grid — the kart ahead
legitimately sits about 2.7 m away there, giving 1.30 m of clearance. Braking at
0 m/s is incoherent: there is nothing to brake for. These tests pin the rule to
stopping distance instead.

Pure arithmetic, no ROS, so it runs on a bare host:
    python3 -m pytest test/test_emergency_decision.py
"""


def is_emergency(clearance, ego_v, emergency_decel=4.0, contact_gap=0.6):
    """Mirror of the rule in CollisionGuard._forward_clearance."""
    stopping = (ego_v * ego_v) / (2.0 * emergency_decel) if ego_v > 0.0 else 0.0
    return clearance <= contact_gap or stopping > max(clearance - contact_gap, 0.0)


# The measured grid geometry: kart 2.70 m ahead, 0.35 m to the side, ego stopped.
GRID_CLEARANCE = 2.70 - 0.9 - 0.5  # other_vehicle_radius + ego_front_offset -> 1.30


def test_stopped_on_the_grid_is_not_an_emergency():
    assert not is_emergency(GRID_CLEARANCE, ego_v=0.0)


def test_creeping_off_the_grid_is_not_an_emergency():
    # 1 m/s creep needs 0.125 m to stop, well inside the 0.70 m usable gap.
    assert not is_emergency(GRID_CLEARANCE, ego_v=1.0)


def test_closing_fast_on_the_same_gap_is_an_emergency():
    # 9 m/s needs 10.1 m to stop and has 0.70 m.
    assert is_emergency(GRID_CLEARANCE, ego_v=9.0)


def test_contact_range_is_an_emergency_even_at_a_standstill():
    assert is_emergency(0.5, ego_v=0.0)


def test_far_obstacle_at_racing_speed_is_not_an_emergency():
    # 9 m/s needs 10.1 m; 20 m of clearance is not an emergency, just a slowdown.
    assert not is_emergency(20.0, ego_v=9.0)


def test_threshold_scales_with_speed_not_distance():
    """The same gap flips from safe to emergency purely on speed."""
    gap = 3.0
    assert not is_emergency(gap, ego_v=2.0)
    assert is_emergency(gap, ego_v=6.0)
