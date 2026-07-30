# collision_guard

Independent **longitudinal safety layer** that sits between the controller (MPC)
and the vehicle. It republishes the controller's `AckermannControlCommand`
unchanged unless another kart or a wall is close ahead in the ego travel
corridor, in which case it **caps the commanded speed** so the ego can always
stop before the obstacle, and — in the worst case — commands an **emergency
brake**. Steering (lateral) is always passed through untouched, so the guard
never fights the controller's racing line.

## Data flow

```
MPC  --/control/command/control_cmd_mpc-->  collision_guard  --/control/command/control_cmd-->  vehicle
                                              ^  /localization/kinematic_state (ego)
                                              ^  /v2x/vehicle_positions        (karts)
                                              ^  /scan                         (walls, optional)
```

Wiring is done in `aichallenge_submit_launch/launch/control/mpc.launch.xml`
(the MPC output is remapped to `.../control_cmd_mpc` and the guard is inserted).
Disable the whole layer with `use_collision_guard:=false`.

## Logic

For each incoming command, the guard finds the nearest obstacle ahead inside a
longitudinal corridor (forward distance > 0, |lateral| < `corridor_half_width`):

- **Karts** from `/v2x/vehicle_positions` (map frame), projected into the ego
  frame. Points within `self_ignore_radius` of the ego are treated as "self".
- **Walls** from `/scan` (narrow dead-ahead cone), used only as a last-resort AEB.

Safe speed = `sqrt(2 * brake_decel * max(clearance - standstill_gap, 0))`.
If the commanded speed exceeds it, the speed is capped and a braking
acceleration is commanded. If clearance < `emergency_gap`, full emergency stop.

## Wall guard is OFF by default

`use_scan_guard` / `use_scan_wall` default to **false**. The only available
`/scan` source in the MPC stack is `laserscan_generator`, which (as wired here)
emits spurious near-zero center readings that false-trigger the AEB — observed as
repeated `EMERGENCY BRAKE: obstacle 0.10 m ahead` during clean laps. The racing
line + MPC already keep the ego off the walls (0 wall collisions in testing), so
the wall branch stays disabled until a clean scan source is available. The V2X
kart guard is geometrically sound and stays enabled.

## Curved corridor (2026-07-29)

The corridor used to be a straight box ahead, which made overtaking impossible:
a kart merely *beside* us, or on the inside of a corner, read as "in path", so the
guard held the car at 0 m/s while the MPC steered around it. Obstacles are now
projected onto the **arc the ego is actually on** (`geometry.project_onto_path`),
so a kart we are going around leaves the corridor as soon as we turn away.

Curvature comes from the measured yaw rate, falling back to the commanded
steering below `curvature_min_speed` — the case that matters most is *stopped in
front of a kart with the controller already steering around it*, where a straight
corridor is exactly wrong.

Three things this exposed, all worth remembering:

- **The MPC's lateral command is a curvature, not an angle.** `u[1]` is
  `tan(delta)/L` and is written straight into `steering_tire_angle`, then scaled by
  `steering_tire_angle_gain`. Converting it with `tan()/wheel_base` is wrong.
- **Constant curvature is only a fair prediction for a limited sweep.** On a tight
  arc the extrapolation curls back into karts beside us and invents an emergency —
  hence `max_sweep_angle`.
- **The vehicle follows the acceleration command.** Capping `speed` while still
  commanding `-brake_decel` pins the car at a standstill. The commanded
  acceleration must agree with the speed cap.

## Status: works on a kart in the way; grid-start-from-rest still fails

Validated 2026-07-30 with `scripts/fake_v2x_publisher.py` (a synthetic V2X kart, so
no second Autoware stack is needed):

| scenario | result |
|----------|--------|
| clear track, avoidance ON | 39.27 / 39.38 / 39.50 s — **no regression** vs 39.5 s median |
| kart parked on the racing line mid-lap (`d2:0:120`) | **8/8 laps**, mean 40.50 s, max 41.09 s, **0 emergency stops**, 12 progressive slowdowns |

Cost of going around a blocking kart: **1.11 s per lap**. The guard's slowdown is
progressive and releases as the MPC steers away — measured on approach: capped to
9.0 m/s at 13.6 m, 8.7 at 12.7, 5.2 at 5.9, 3.7 at 3.9, then through.

**Still failing:** two karts parked across the track *at the grid*, with the ego
starting from **rest** behind them (`SIM_MODE=dev3`, Autoware on d1 only) — 0 laps.
That the same car now clears a parked kart mid-lap at racing speed strongly
suggests the failure is specific to planning from a standstill: this spatial MPC
parametrises by path position, so with no forward motion the corridor narrowing
never yields an avoidance path. Production starts all karts moving off together,
so this is a lower-priority gap — but it is a real one, e.g. after a spin.

## Earlier status (superseded)

The lateral avoidance path (`use_obstacle_avoidance`, now defaulted **on**) has
**not** been shown to work end to end. In the hardest test — two karts parked
across the track at the grid (`SIM_MODE=dev3`, Autoware on d1 only) — the ego now
creeps and steers instead of deadlocking, but still ends up emergency-stopping and
completed **0 laps**. Do not treat this as race-ready. Open questions:

- **The MPC does not steer away at all in this scenario, and that is the real
  failure** — the emergency stop is a symptom. Measured: ego frozen at 0 m/s with
  a kart 2.7 m ahead and only 0.69 m off its centreline, i.e. genuinely aimed at
  it, so the emergency is *correct*. Suspect the standing start: this spatial MPC
  parametrises by path position and needs forward motion to plan, so the corridor
  narrowing may never produce an avoidance path from rest. Not confirmed.
- The parked-at-the-grid case is harsher than production, where all karts move off
  together. **The realistic moving-traffic test could not be run on this machine**:
  two Autoware stacks plus AWSIM saturate 8 cores (load average 12-19, both MPC
  processes pinned at ~96%), and neither kart completed a lap — that measures CPU
  starvation, not control. Killing rviz did not recover enough headroom.
- **Recommended next step:** test with a *synthetic* V2X publisher — a small node
  that injects a virtual kart moving along `traj_mincurv.csv` at a chosen speed.
  No second Autoware stack, so no CPU problem, and it is deterministic and
  reproducible. That is the harness this feature actually needs.
- No clear-track regression run yet: avoidance costs 6.9 ms mean / 10.3 ms p95 of
  the 25 ms control budget (`scripts/bench_avoidance.py`), so the sub-40 s lap time
  needs re-confirming with it enabled.

## Key parameters (`config/collision_guard.param.yaml`)

| param | meaning |
|-------|---------|
| `brake_decel` | decel used to compute the safe speed |
| `standstill_gap` | gap at which target speed reaches 0 |
| `emergency_gap` | clearance below which a full stop is commanded |
| `emergency_half_width` | ...but only for obstacles this close to the path centreline |
| `corridor_half_width` | lateral half-width of the corridor, measured off the arc |
| `curved_corridor` | project obstacles onto the ego's arc instead of a straight box |
| `max_sweep_angle` | ignore obstacles further than this far around the arc |
| `creep_speed` | floor on the capped speed, so the car can never be stranded |
