"""Decision logic for the stuck-recovery filter.

Deliberately pure Python with no ROS imports, mirroring
``collision_guard.geometry``: the state machine and the mirrored steering are the
parts most likely to hide a sign or timing error, and keeping them importable on
a bare host makes it cheap to unit-test.

Two things live here:

* :class:`StuckRecovery` -- when to take over from the controller, and which
  phase of the manoeuvre we are in.
* :func:`recovery_steer` / :func:`path_target` -- where the reference path is
  relative to the ego, and which way to point the wheels to get back onto it.
"""

import math
from dataclasses import dataclass

# --- phases ---
IDLE = "idle"
SHIFT_REVERSE = "shift_reverse"
REVERSE = "reverse"
SHIFT_DRIVE = "shift_drive"
FORWARD = "forward"
COOLDOWN = "cooldown"

# Phases in which the node overrides the controller's command. COOLDOWN is not
# one of them: the controller drives again immediately, the cooldown only keeps
# the detector from re-triggering before it has had a chance to.
ACTIVE_PHASES = (SHIFT_REVERSE, REVERSE, SHIFT_DRIVE, FORWARD)
# Phases that need the vehicle in reverse gear.
REVERSE_PHASES = (SHIFT_REVERSE, REVERSE)


def wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def recovery_steer(lat: float, e_psi: float, k_lat: float, k_psi: float,
                   max_steer: float) -> float:
    """Steering command that turns the nose back onto the reference path.

    ``lat`` is how far to the left of the ego the path target sits, ``e_psi`` the
    heading error (path direction minus ego heading). The result is the command
    for *driving forward*; while reversing the caller must mirror it (see
    :func:`mirrored_for_reverse`).
    """
    return max(-max_steer, min(max_steer, k_psi * e_psi + k_lat * lat))


def mirrored_for_reverse(steer: float) -> float:
    """The same correction, executed backwards.

    Yaw rate is ``v * tan(delta) / L``, so with ``v < 0`` the very same steering
    angle rotates the car the other way. Reversing with the mirrored angle is
    what swings the nose *towards* the path instead of further off it.
    """
    return -steer


def path_target(points, x: float, y: float, yaw: float, lookahead: float):
    """Locate the reference path relative to the ego.

    ``points`` is a sequence of ``(px, py)`` in the same frame as ``x``/``y``.
    Returns ``(lat, e_psi)``: how far to the left of the ego the target point
    sits, and the heading error at the closest point. Returns ``None`` when the
    path is too short to say anything.

    The target is taken ``lookahead`` metres further along the path than the
    closest point, so a car sitting *on* the line but pointing at the wall still
    gets a meaningful correction.
    """
    n = len(points)
    if n < 2:
        return None

    nearest = min(range(n), key=lambda i: (points[i][0] - x) ** 2 + (points[i][1] - y) ** 2)

    # A raceline is a closed loop, so walking off the end wraps. Only treat it as
    # closed when the ends actually meet; otherwise clamp, or an open path would
    # send the car back to the start line.
    closed = math.hypot(points[0][0] - points[-1][0], points[0][1] - points[-1][1]) < 5.0

    j = nearest
    travelled = 0.0
    while travelled < lookahead:
        k = j + 1
        if k >= n:
            if not closed:
                break
            k = 0
        travelled += math.hypot(points[k][0] - points[j][0], points[k][1] - points[j][1])
        j = k
        if j == nearest:  # walked the whole loop
            break

    tx, ty = points[j]
    rx, ry = tx - x, ty - y
    lat = -rx * math.sin(yaw) + ry * math.cos(yaw)

    # Path direction at the closest point, which is what the ego has to line up
    # with -- not the direction at the (possibly far away) target.
    nxt = nearest + 1
    if nxt >= n:
        nxt = 0 if closed else nearest
        if nxt == nearest:
            nearest -= 1
            nxt = n - 1
    path_yaw = math.atan2(points[nxt][1] - points[nearest][1],
                          points[nxt][0] - points[nearest][0])
    return lat, wrap_to_pi(path_yaw - yaw)


@dataclass
class RecoveryConfig:
    """Timings and thresholds of the manoeuvre. Defaults mirror the param file."""

    # --- detection ---
    # Never fire before the car has driven once: on the grid it legitimately sits
    # still for seconds while the controller already commands a speed, and
    # reversing off the grid before the start would be a self-inflicted 0 laps.
    arm_speed: float = 2.0
    # "No progress" is the real test. A wedged kart can still creep along a wall
    # at 0.1 m/s, and a car spinning its wheels is moving without going anywhere.
    stuck_radius: float = 1.0
    stuck_duration: float = 2.5
    # ...and only while the controller actually wants to move, so a commanded
    # stop is never mistaken for being stuck.
    intent_speed: float = 0.5

    # --- manoeuvre ---
    # Measured on the 2026-08-03 AWSIM: reverse speed saturates at 1.38 m/s after
    # a ~1.9 s ramp, so 3 m of backing out fits inside the timeout and 4 m did not.
    shift_timeout: float = 0.5
    reverse_distance: float = 3.0
    reverse_timeout: float = 4.0
    forward_duration: float = 2.0
    # Hand back to the controller as soon as the car is genuinely rolling; it
    # drives the line better than this node ever will.
    forward_release_speed: float = 2.0
    cooldown: float = 3.0

    # --- giving up ---
    # Three, despite each retry landing inside AWSIM's ~12.2 s penalty union
    # window: cutting to one was measured worse overall (dashboard R68), because
    # the retries are also what frees the car. See the param file.
    max_attempts: int = 3
    give_up_cooldown: float = 10.0
    attempt_reset_distance: float = 15.0


class StuckRecovery:
    """Decides when the recovery takes over and which phase it is in.

    Feed it :meth:`update` once per control command; read :attr:`phase` and
    :meth:`is_active` to build the outgoing command.
    """

    def __init__(self, cfg: RecoveryConfig) -> None:
        self.cfg = cfg
        self.phase = IDLE
        self.attempts = 0
        self.armed = False
        self._anchor = None          # position the no-progress timer is measured from
        self._anchor_t = 0.0
        self._phase_t = 0.0          # when the current phase started
        self._phase_origin = (0.0, 0.0)
        self._release_pos = None     # where the last recovery handed control back

    # --- queries ---
    def is_active(self) -> bool:
        """True while the node is driving instead of the controller."""
        return self.phase in ACTIVE_PHASES

    def wants_reverse_gear(self) -> bool:
        return self.phase in REVERSE_PHASES

    # --- main entry point ---
    def update(self, t: float, x: float, y: float, v: float,
               wants_to_move: bool, gear_ready: bool, rear_blocked: bool) -> str:
        """Advance the state machine and return the phase to act on."""
        if abs(v) >= self.cfg.arm_speed:
            self.armed = True

        if self.phase == IDLE:
            self._update_idle(t, x, y, wants_to_move, rear_blocked)
        elif self.phase == SHIFT_REVERSE:
            if gear_ready or self._elapsed(t) >= self.cfg.shift_timeout:
                self._enter(REVERSE, t, x, y)
        elif self.phase == REVERSE:
            backed = math.hypot(x - self._phase_origin[0], y - self._phase_origin[1])
            if (backed >= self.cfg.reverse_distance
                    or self._elapsed(t) >= self.cfg.reverse_timeout
                    or rear_blocked):
                self._enter(SHIFT_DRIVE, t, x, y)
        elif self.phase == SHIFT_DRIVE:
            if gear_ready or self._elapsed(t) >= self.cfg.shift_timeout:
                self._enter(FORWARD, t, x, y)
        elif self.phase == FORWARD:
            if self._elapsed(t) >= self.cfg.forward_duration or v >= self.cfg.forward_release_speed:
                self._release_pos = (x, y)
                self._enter(COOLDOWN, t, x, y)
        elif self.phase == COOLDOWN:
            if self._elapsed(t) >= self._cooldown_length():
                if self.attempts >= self.cfg.max_attempts:
                    self.attempts = 0  # served the long cooldown, start over
                self._enter(IDLE, t, x, y)
                self._reset_anchor(t, x, y)

        self._maybe_forget_attempts(x, y)
        return self.phase

    # --- internals ---
    def _update_idle(self, t: float, x: float, y: float,
                     wants_to_move: bool, rear_blocked: bool) -> None:
        if self._anchor is None:
            self._reset_anchor(t, x, y)
            return

        moved = math.hypot(x - self._anchor[0], y - self._anchor[1])
        if moved > self.cfg.stuck_radius or not wants_to_move:
            self._reset_anchor(t, x, y)
            return
        if not self.armed:
            return
        if (t - self._anchor_t) < self.cfg.stuck_duration:
            return

        self.attempts += 1
        # Backing into a kart sitting behind us would trade one problem for a
        # penalty, so when the way back is blocked we only try the forward half.
        self._enter(SHIFT_DRIVE if rear_blocked else SHIFT_REVERSE, t, x, y)

    def _cooldown_length(self) -> float:
        if self.attempts >= self.cfg.max_attempts:
            return self.cfg.give_up_cooldown
        return self.cfg.cooldown

    def _maybe_forget_attempts(self, x: float, y: float) -> None:
        """A recovery that led to real driving does not count against the next one."""
        if self._release_pos is None:
            return
        if math.hypot(x - self._release_pos[0], y - self._release_pos[1]) \
                > self.cfg.attempt_reset_distance:
            self.attempts = 0
            self._release_pos = None

    def _elapsed(self, t: float) -> float:
        return t - self._phase_t

    def _enter(self, phase: str, t: float, x: float, y: float) -> None:
        self.phase = phase
        self._phase_t = t
        self._phase_origin = (x, y)

    def _reset_anchor(self, t: float, x: float, y: float) -> None:
        self._anchor = (x, y)
        self._anchor_t = t
