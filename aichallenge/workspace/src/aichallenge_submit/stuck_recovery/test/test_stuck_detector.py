"""When the recovery is allowed to take over, and what it does once it has.

The expensive mistake here is a false positive: reversing mid-race, or off the
starting grid before the lights, costs far more than staying stuck. These tests
pin the two guards that prevent it (arming, and the controller's intent) and the
phase order of the manoeuvre itself.

Pure arithmetic, no ROS, so it runs on a bare host:
    python3 -m pytest test/test_stuck_detector.py
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from stuck_recovery.recovery_logic import (  # noqa: E402
    COOLDOWN, FORWARD, IDLE, REVERSE, SHIFT_DRIVE, SHIFT_REVERSE,
    RecoveryConfig, StuckRecovery,
)


def make():
    return StuckRecovery(RecoveryConfig())


def drive(fsm, t0, seconds, x=0.0, y=0.0, v=0.0, wants=True,
          gear_ready=True, rear_blocked=False, step=0.025):
    """Hold the car at one place (or state) for `seconds` and return the end time."""
    t = t0
    while t < t0 + seconds:
        fsm.update(t, x, y, v, wants, gear_ready, rear_blocked)
        t += step
    return t


def until_phase(fsm, t0, phase, x=0.0, y=0.0, v=0.0, wants=True,
                gear_ready=True, rear_blocked=False, step=0.025, limit=60.0):
    """Tick until the machine reaches `phase`; assert it does. Returns the time."""
    t = t0
    while t < t0 + limit:
        if fsm.update(t, x, y, v, wants, gear_ready, rear_blocked) == phase:
            return t
        t += step
    raise AssertionError(f"never reached {phase} (stuck in {fsm.phase})")


def arm(fsm, t=0.0):
    """Drive once, which is what arms the recovery."""
    fsm.update(t, 0.0, 0.0, 5.0, True, True, False)


# --- the two false-positive guards ---

def test_standing_on_the_grid_never_triggers():
    # The controller commands a speed the whole time, the car does not move, and
    # it has never driven: this is the starting grid, not a stuck car.
    fsm = make()
    drive(fsm, 0.0, 30.0, wants=True)
    assert fsm.phase == IDLE
    assert not fsm.armed


def test_commanded_stop_never_triggers():
    fsm = make()
    arm(fsm)
    drive(fsm, 0.0, 30.0, wants=False)
    assert fsm.phase == IDLE


def test_a_car_that_keeps_moving_never_triggers():
    fsm = make()
    arm(fsm)
    t = 0.0
    for i in range(2000):  # 50 s of driving, 0.2 m per tick
        fsm.update(t, 0.2 * i, 0.0, 8.0, True, True, False)
        t += 0.025
    assert fsm.phase == IDLE


# --- detection ---

def test_wedged_against_a_wall_triggers_after_stuck_duration():
    fsm = make()
    arm(fsm)
    # Creeping along the wall at 0.1 m/s: moving, but going nowhere.
    t = drive(fsm, 0.0, 2.0, v=0.1)
    assert fsm.phase == IDLE, "must not fire before stuck_duration"
    t = until_phase(fsm, t, SHIFT_REVERSE, v=0.1)
    assert t < 3.0
    assert fsm.attempts == 1


def test_progress_resets_the_timer():
    fsm = make()
    arm(fsm)
    t = 0.0
    # Stuck for 2 s, then a 2 m lurch, then stuck for 2 s again: never 2.5 s
    # without progress, so no recovery.
    t = drive(fsm, t, 2.0, x=0.0)
    fsm.update(t, 2.0, 0.0, 1.0, True, True, False)
    drive(fsm, t, 2.0, x=2.0)
    assert fsm.phase == IDLE


# --- the manoeuvre ---

def test_full_sequence_reverse_then_drive_then_hand_back():
    fsm = make()
    arm(fsm)
    t = until_phase(fsm, 0.0, SHIFT_REVERSE)

    # Shift completes as soon as the gear report agrees.
    fsm.update(t, 0.0, 0.0, 0.0, True, True, False)
    assert fsm.phase == REVERSE
    assert fsm.wants_reverse_gear()
    assert fsm.is_active()

    # Backed the full reverse_distance -> shift back to drive.
    t += 1.0
    fsm.update(t, -4.5, 0.0, -2.0, True, True, False)
    assert fsm.phase == SHIFT_DRIVE
    assert not fsm.wants_reverse_gear()

    t += 0.1
    fsm.update(t, -4.5, 0.0, 0.0, True, True, False)
    assert fsm.phase == FORWARD

    # Rolling again -> control goes back to the controller immediately.
    t += 0.1
    fsm.update(t, -4.0, 0.0, 2.5, True, True, False)
    assert fsm.phase == COOLDOWN
    assert not fsm.is_active()


def test_reverse_gives_up_on_timeout_even_without_moving():
    fsm = make()
    arm(fsm)
    t = until_phase(fsm, 0.0, REVERSE)
    # Wedged so hard it does not even reverse: the timeout still ends the phase.
    t = until_phase(fsm, t, SHIFT_DRIVE)
    assert fsm.phase == SHIFT_DRIVE


def test_shift_does_not_wait_forever_for_a_gear_report():
    # No gear feedback at all (no GearReport publisher) must not deadlock the
    # manoeuvre: the shift timeout carries it through.
    fsm = make()
    arm(fsm)
    t = until_phase(fsm, 0.0, SHIFT_REVERSE, gear_ready=False)
    t = until_phase(fsm, t, REVERSE, gear_ready=False)
    assert t < 4.0


def test_blocked_rear_skips_the_reverse_half():
    fsm = make()
    arm(fsm)
    until_phase(fsm, 0.0, SHIFT_DRIVE, rear_blocked=True)
    assert not fsm.wants_reverse_gear()


def test_cooldown_prevents_immediate_retrigger():
    fsm = make()
    arm(fsm)
    t = until_phase(fsm, 0.0, COOLDOWN)
    # Still not moving, still asking to move: the cooldown must hold anyway.
    t = drive(fsm, t, 2.0)
    assert fsm.phase == COOLDOWN


def test_attempts_stop_after_the_limit_and_resume_later():
    cfg = RecoveryConfig()
    fsm = StuckRecovery(cfg)
    arm(fsm)
    t = 0.0
    for attempt in range(1, cfg.max_attempts + 1):
        t = until_phase(fsm, t, COOLDOWN)
        assert fsm.attempts == attempt
        last_cooldown_start = t
        if attempt < cfg.max_attempts:
            t = drive(fsm, t, cfg.cooldown + 0.1)

    # Attempt limit reached: the long cooldown is in force, so a car that is still
    # stuck is left to the controller instead of shuttling back and forth forever.
    t = drive(fsm, t, cfg.cooldown + 0.5)
    assert fsm.phase == COOLDOWN, "the normal cooldown must not end the give-up wait"

    t = until_phase(fsm, t, IDLE, limit=cfg.give_up_cooldown + 1.0)
    assert (t - last_cooldown_start) >= cfg.give_up_cooldown
    assert fsm.attempts == 0, "the count is cleared, so a later incident is tried again"


# AWSIM adds ~12.2 s to a live contact penalty for every further contact, so a
# retry inside that window is charged as an extension of the expensive event
# rather than a cheap new one. Measured across 35 penalty events.
PENALTY_UNION_WINDOW_S = 12.2


def test_retries_are_known_to_land_inside_the_penalty_window():
    """Documents a trade that was measured, not an invariant to preserve.

    Each retry follows the last by roughly reverse + forward + cooldown +
    stuck_duration, which is under AWSIM's penalty union window, so a re-contact
    extends the current penalty instead of starting a cheap new one. Cutting to a
    single attempt fixed that and still lost on lap time (dashboard R68): the
    retries are also what frees the car. Kept at three deliberately.
    """
    cfg = RecoveryConfig()
    assert cfg.max_attempts == 3
    cycle = cfg.reverse_timeout + cfg.forward_duration + cfg.cooldown + cfg.stuck_duration
    assert cycle < PENALTY_UNION_WINDOW_S + 3.0, \
        "if this ever grows past the window, revisit R68 — the trade may flip"


def test_driving_away_clears_the_attempt_count():
    fsm = make()
    arm(fsm)
    t = until_phase(fsm, 0.0, COOLDOWN)
    assert fsm.attempts == 1
    # 20 m further down the track, the next incident is a fresh one.
    fsm.update(t + 1.0, 20.0, 0.0, 8.0, True, True, False)
    assert fsm.attempts == 0
