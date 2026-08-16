"""Deciding which way to go around a kart, and how hard to steer to do it.

Pure Python, no ROS, mirroring ``collision_guard.geometry`` and
``stuck_recovery.recovery_logic``: the side choice and the curvature conversion
are where a sign error would hide, and keeping them importable on a bare host
makes them cheap to test.

    python3 -m pytest test/

The one thing to understand before reading: **the vehicle's lateral command is a
curvature, not an angle.** ``AckermannControlCommand.lateral.steering_tire_angle``
carries ``kappa * steering_tire_angle_gain`` (the MPC writes ``tan(delta)/L``
there and the interface scales it), so adding a bias means adding
``delta_kappa * gain`` — never an angle, and never ``tan()/wheel_base``.
"""

import math


def pass_side(kart_lat, ego_offset, pass_clearance, max_offset):
    """Which side to pass on, and the lateral shift it needs. None = stay put.

    ``kart_lat`` is where the kart sits in the ego frame (+ left), ``ego_offset``
    how far the ego already is from the racing line (+ left), both metres.

    Returns the signed shift to add to ``ego_offset`` (+ left), or None when the
    kart is already clear or neither side has room. Preference goes to the
    cheaper side, but a side that would push the car off the track is not
    available at any price — leaving the surface is worse than following.
    """
    if abs(kart_lat) >= pass_clearance:
        return 0.0  # already passing wide enough; hold the line

    # To clear on the right the kart must end up pass_clearance to our LEFT, so
    # we move right by the shortfall; mirrored for the left.
    shift_right = -(pass_clearance - kart_lat)
    shift_left = pass_clearance + kart_lat

    options = []
    for shift in (shift_right, shift_left):
        if abs(ego_offset + shift) <= max_offset:
            options.append(shift)
    if not options:
        return None
    return min(options, key=abs)


def curvature_for_shift(shift, preview):
    """Curvature that moves the car sideways by ``shift`` over ``preview`` metres.

    Small-angle arc: a constant curvature kappa displaces kappa*d^2/2 laterally
    after distance d, so kappa = 2*shift/d^2.
    """
    if preview <= 0.0:
        return 0.0
    return 2.0 * shift / (preview * preview)


def steering_bias(shift, preview, gain, max_curvature):
    """The number to ADD to ``steering_tire_angle`` to achieve ``shift``.

    Clamped in curvature space, because that is where the vehicle limit lives;
    clamping the scaled command instead would make the limit depend on the gain.
    """
    kappa = curvature_for_shift(shift, preview)
    kappa = max(-max_curvature, min(max_curvature, kappa))
    return kappa * gain


def blend(previous, target, rate):
    """Move ``previous`` towards ``target`` by at most ``rate``.

    The bias is rate-limited rather than applied instantly: a step change in
    steering at racing speed is both a spike the vehicle cannot follow and, if
    the kart flickers in and out of the corridor, an oscillation.
    """
    delta = target - previous
    if delta > rate:
        delta = rate
    elif delta < -rate:
        delta = -rate
    return previous + delta
