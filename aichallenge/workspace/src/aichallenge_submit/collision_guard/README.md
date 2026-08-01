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

### Small speed differential: the ego gets stuck behind, and the MPC is the cause

Tested with an opponent at 28 km/h — under 1 m/s slower than the ego's 31.4 km/h
average, so the two run together for a long time (`d2:28:20`):

| run | result |
|-----|--------|
| first | 3 laps at **42.17 / 42.50 / 44.75 s**, 71 slowdowns, 0 emergencies — the ego *trails* rather than passing, losing 3-5 s/lap |
| second | 1 lap, then **stopped dead and never restarted** |

The stop is **not** the guard: it was capping to 3.9-4.0 m/s while the ego sat at
0.00 m/s, i.e. permitting four times the speed the car was doing. The command chain
shows the MPC itself commanding the stop, and its own logs say why —
**111 × `Relaxed safety margin ... to solve the problem`** and **7 ×
`Infeasible path detected`**, the branch in `update_path_constraints` that pins
`ub_sm = lb_sm = 0.0`, i.e. a zero-width corridor.

So the corridor narrowing turns infeasible when a kart is close ahead at low speed,
the QP degenerates, and the MPC outputs a stop it cannot recover from. This is the
same failure as the grid start below — **planning from (near) standstill with an
obstacle in the corridor** — and it is now the single blocking defect for racing in
traffic. It is an MPC/corridor problem, not a guard problem.

#### Root cause found and fixed: the safety margin made passing geometrically impossible

`BicycleModel.safety_margin` was hard-coded to `width / sqrt(2)` = **1.202 m**, applied
to *each* side on top of the 1.70 m width. Using a gap therefore demanded
`1.70 + 2 x 1.202 = 4.1 m` of free corridor, but the gap either side of a kart on the
racing line of a ~6 m track is only about **2.5 m**. The arithmetic in
`add_constraint` landed on `segment_length_sm = 2.5 - 2.40 = 0.10 m`, exactly the
`min_segment_length` threshold — so the corridor flickered between "usable" and
"infeasible", which is what collapsed it to zero width and stalled the car.

`safety_margin` is now a `bicycle_model` config key (default unchanged at
`width/sqrt(2)`), set to **0.85** = half the real 1.45 m body plus ~0.13 m clearance.
That turns the 0.10 m knife-edge into a 0.80 m margin. The zero-width collapse also
now degrades to the static track bounds instead of pinning `e_y` to exactly 0.

**Measured effect (2026-08-01):**

| scenario | before | after |
|----------|--------|-------|
| clear track | 39.3-39.5 s | **39.41-39.85 s over 6 laps — no regression** |
| opponent at 28 km/h | 42.17 / 42.50 / 44.75 s, 71 slowdowns, then **permanent stall** | 39.30-39.61 s over 5 laps, 0 slowdowns, **no stall** |
| opponent at 26 km/h, injected 12 m ahead | — | 44.31 / 48.12 / 48.23 s, 73 slowdowns, 1 emergency, **no stall** |

So the unrecoverable stall is fixed and the car keeps racing. **But it still does not
overtake.** In the close-quarters run the ego spent 75.9 s within 12 m of the
opponent and never got closer than **9.58 m** centre-to-centre, averaging 27.4 km/h
against 31.4 clear — it sits behind and loses ~8.7 s/lap. (The third row's opponent
was close enough to matter; the second row's was never caught within 14 m, which is
why it shows no loss — do not read that row as evidence of a clean pass.)

Remaining work is making the pass actually happen: the guard's speed cap
(`standstill_gap`, `brake_decel`) holds the ego ~10 m back, and at that range the
MPC's corridor narrowing is not producing a decisive line change. Worth trying: a
shorter following distance so the ego is close enough for the corridor to bite, and
biasing the reference laterally when a kart sits on the line ahead.

**Do not attempt** re-resetting `ub_sm`/`lb_sm` per control cycle to break the
ratchet described in `update_path_constraints` without also resetting
`dynamic_border_cells`: tried on 2026-07-31, and the car stalled on lap 3 of a
*clear* track.

**Also still failing:** two karts parked across the track *at the grid* with the ego
starting from rest (`SIM_MODE=dev3`, Autoware on d1 only) — 0 laps. Same root cause.

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
