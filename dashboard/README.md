# Racing Kart — Run Breakdown Dashboard

One block per simulator start: **how many laps it completed, and what happened on
each of them**. Auto-parsed from run logs.

> Reset on 2026-08-11 to this single view. It previously also carried KPI tiles, an
> auto-derived findings panel, a config-comparison table and a lap-time chart; those
> were removed. `parse_runs.py` still extracts the config fields they used and they
> are still in `run_data.json`, so any of it can be rebuilt from the parsed payload
> without re-running anything.

## Files
- `parse_runs.py` — scans `output/*/d*/autoware.log`, extracts each run's config,
  lap times and guard activity, and injects the result into `dashboard.html`.
- `dashboard.html` — self-contained page (open in a browser or publish as an Artifact).
  Reads its data from an embedded `<script id="run-data">` block that the parser rewrites.
- `lap_detail.py` — per-lap breakdown: which lap an event belongs to, and what
  position we held at a given moment. Pure arithmetic, no file formats.
- `test_lap_detail.py` — its tests (`python3 -m pytest dashboard/test_lap_detail.py`).
- `run_data.json` — the parsed payload (also written out for reuse).

## The view
Newest run open, the rest collapsed. Each block headlines the **session timestamp**
and **laps completed** against the session's own requirement, then gives a row per
lap: time, kart contacts, wall contacts, recoveries, over-speed penalties, guard
slowdowns, the position held while speed-limited, and penalty seconds. It closes
with why the run did not finish.

### What a run has to produce to fill it in
| Column | Needs | If missing |
|---|---|---|
| lap time, laps completed | `result-summary.json` or `result-details.json` | falls back to the controller's log |
| kart / wall / over-speed contacts, penalty s | **`result-details.json`** | shown as `n/r`, *never* as 0 |
| finished, required laps, final position | `result-summary.json` | "no judge record" |
| recoveries, guard slow | `autoware.log` | always present |
| position while speed-limited | **`Player.log`** | shown as `—` |

Only the **evaluation bundle** writes `result-details.json`. A run started as
`make simulator-<mode>` + `make autoware-simulator` has no contact data at all —
which is why `n/r` exists and why reading a blank as "no collisions" is a mistake
that has already been made once here. To get a full row:

```bash
SIM_MODE=race CMD='/aichallenge/run_evaluation.bash' docker compose run -d --name aic-eval-race autoware-command
# ...after it finishes, before removing the container:
tools/save_sim_artifacts.sh aic-eval-race output/<ts>/d1
```

`save_sim_artifacts.sh` copies out `Player.log`, which is the only record of the
*opponents'* lap crossings and therefore the only way to reconstruct our position
during the race — AWSIM exposes nothing but `final_position` anywhere else. It never
overwrites a file that is already there; `/aichallenge` is a shared bind mount and the
`result-summary.json` sitting in it can belong to an older race.

## Refresh after a run
```bash
python3 dashboard/parse_runs.py            # update dashboard.html in place
python3 dashboard/parse_runs.py --print    # ...and print the JSON
python3 dashboard/parse_runs.py --min-laps 2 --since 20260719   # focus recent, drop stubs
python3 dashboard/parse_runs.py --target-s 40   # per-lap time target (default 40 s)
```
Then reload `dashboard.html` (or re-publish the Artifact).

## What it extracts (from `autoware.log`)
| Field | Source line |
|-------|-------------|
| `v_max`, `a_max`, `a_min`, `ay_max`, `width`, `safety_margin`, `wp_id_offset` | MPC config echo (`ay_max: 19.0` …) |
| `Q[0]` | first element of the `Q: [...]` config line |
| `use_speed_profile` | `use_speed_profile: true` (shown as `fwd-bwd` vs `kappa-pred`) |
| `avoid` | `USE_OBSTACLE_AVOIDANCE is enabled` — a launch arg, so it only appears as the controller's startup warning |
| `traffic` | any `traffic: kart` / `obstacle points in corridor` line, i.e. whether another kart was actually on track |
| `ref_vel` corners (s4/s6/s8) | `ref_vel:` lines (indices 4/6/8) |
| guard state | `collision_guard up (v2x=…, scan=…)` |
| guard events | `EMERGENCY BRAKE`, `slow: cap` counts |
| recovery state / count | `stuck_recovery up (enabled=…)`, `STUCK detected` count |
| lap times | `Lap N completed! Lap time: X s` (+ ROS stamp) |
| **live param changes** | `<param> was updated to '<value>'` (+ ROS stamp) |
| **change vs previous** | auto-diff of config against the previous run |
| **result** | `ok` / `best` / `partial` / `fail` (stall heuristic) |

The config fields are no longer displayed anywhere; they stay in `run_data.json`
because they cost nothing to keep and the comparison view may come back.

Commented-out preset blocks in `config.yaml` are echoed to the log too, but they
carry a leading `#`, so only the live values are picked up.

### Lap times: AWSIM's record beats the controller's log
In a **race session** (NPCs, ranking on) the number AWSIM hands the controller for
`Lap N completed! Lap time:` is the **cumulative** session time, not the lap — one
6-lap race logged 110/215/275/327/377/425 s where AWSIM's own record said
107/107/61/50/50/50 s. So if a `result-summary.json` sits next to `autoware.log`,
the parser takes lap times from it instead. **After a race run, save it**:

```bash
docker exec aichallenge-racingkart-simulator-1 cat /aichallenge/result-summary.json \
  > output/<ts>/d1/result-summary.json
```

For an old run where it was never saved, put the real per-lap times in
`run_meta.json` as `"laps": [...]` and say in the `note` where they came from.
Solo `eval.sh` runs are unaffected — there the logged value is the lap.

`best` = fastest flying lap of the session (lap 1 = standing start, excluded).
`fail` = completed < target *and* the sim kept running well past the last lap
(stall / off-line), e.g. the v_max 34 experiment.

### One log can become several rows
Limits are tunable at runtime (`ros2 param set /mpc_controller ay_max 13.0`), and a
long run where `ay_max` was walked 10 → 13 → 16 → 19 is **four experiments, not one**.
The parser splits each log at the moments the node logged an accepted change and
emits one row per config segment, suffixed `· a`, `· b`, … with `(live)` on the
change text. Laps are assigned by completion time, so the first lap of a segment
straddles the change — it is dropped from that segment's best, exactly like a
standing-start lap. Note the node applies a param **2-3 laps after** the
`param set` returns, so trust the log line, not the command.

Rows are ordered by the log's own first ROS timestamp, not by directory name, so a
hand-named `LOG_DIR` (`output/20260726-w170`) still lands in the right place.

### Clear laps vs traffic laps are never mixed
A run with another kart on track is not comparable pace, so the `traffic` flag keeps
them apart. Nothing on the page pools them now, but honour it in anything built on
`run_data.json`.

### Implausible laps are dropped
Laps under `MIN_PLAUSIBLE_LAP_S` (30 s) are discarded as simulator artefacts. The
raceline is ~345 m and the kart tops out at 37 km/h, so even flat out a lap takes
33.6 s; a 20 s lap appeared once after a wall recovery and would otherwise have
become the headline "best lap" and the largest apparent improvement.

## Manual labels and overrides — `run_meta.json`
Drop one next to the log (`output/<ts>/d1/run_meta.json`):
```json
{ "label": "my name", "change": "custom note", "blocker": "what really stopped it",
  "laps": [64.8, 108.4], "result": "partial", "note": "why",
  "exclude": false, "keep": true }
```
- `blocker` — override the derived "completion blocked by" line, for a reason only a
  human watched happen. `laps` — real per-lap times for an old run whose
  `result-summary.json` was never kept.
- `change` / `label` — override the auto-derived text (`change` applies to the first segment).
- `result` — override the classifier, e.g. a run AWSIM ended on its own is not a
  control `fail`. `note` — free text carried into the JSON.
- `exclude` — drop this run. `keep` — force-keep a short run past `--min-laps`,
  **including a 0-lap one** (otherwise dropped as a boot-only stub). Use it for a run
  that failed to complete laps on purpose-of-record, e.g. a start that beached.

## Publishing as an Artifact (claude.ai)
`dashboard.html` is CSP-safe (no external assets). Publish it to get a private,
shareable URL; re-publish the same file to update it in place.
