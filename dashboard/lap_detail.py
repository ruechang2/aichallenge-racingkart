"""Per-lap breakdown of a run: what happened, on which lap, and in which position.

Kept apart from ``parse_runs.py`` and free of any file-format guessing so the two
things that are easy to get wrong -- attributing an event to the right lap, and
reconstructing our position at a moment in the race -- can be exercised directly:

    python3 -m pytest dashboard/test_lap_detail.py

Three clocks meet here and they are not the same clock:

* **race time** -- seconds since the green flag. AWSIM's ``result-details.json``
  timestamps its penalties this way, and lap times sum to it.
* **ROS time** -- what ``autoware.log`` stamps. Only its *differences* mean
  anything to us, so it is converted to race time via the lap the two agree on.
* **lap number** -- 1-based, the way both AWSIM and a human count laps.
"""

import re

# AWSIM's own record of who crossed the line when. Player.log is the only place
# the opponents appear at all, and it is inside the simulator container, so it has
# to be copied out before the container goes away (tools/save_sim_artifacts.sh).
RE_EGO_LAP = re.compile(r'^Lap completed:\s*([\d.]+)s,\s*total laps:\s*(\d+)', re.M)
RE_NPC_LAP = re.compile(r'^\[NpcLap\]\s*(\S+)\s+lap\s+(\d+):\s*([\d.]+)s', re.M)

# The kinds AWSIM scores, as they appear in result-details.json.
PENALTY_KINDS = ('crash', 'wall', 'over')


def _cumulative(lap_times):
    """Lap times -> the race time at which each of those laps was completed."""
    out, t = [], 0.0
    for lap in lap_times:
        t += lap
        out.append(t)
    return out


def parse_player_log(text):
    """Crossing times (race time, seconds) for us and for every NPC.

    Both line formats report a *lap* time, not a running total, so they are
    accumulated. Returns ``(ego_crossings, {npc_name: crossings})``.
    """
    ego = _cumulative([float(t) for t, _ in RE_EGO_LAP.findall(text)])

    per_npc = {}
    for name, lap_no, lap_time in RE_NPC_LAP.findall(text):
        per_npc.setdefault(name, []).append((int(lap_no), float(lap_time)))
    npcs = {}
    for name, entries in per_npc.items():
        entries.sort()
        npcs[name] = _cumulative([t for _, t in entries])
    return ego, npcs


def rank_at(race_time, ego_crossings, npc_crossings):
    """Our position at ``race_time``, or None if there is nothing to compare to.

    A car is ahead of us when it has completed more laps than we have, or the
    same number but got there first. Ties on lap count are broken by crossing
    time because that is the order the cars are actually in.
    """
    if not npc_crossings:
        return None

    def laps_and_last(crossings):
        done = [t for t in crossings if t <= race_time]
        return len(done), (done[-1] if done else None)

    our_laps, our_last = laps_and_last(ego_crossings)
    ahead = 0
    for crossings in npc_crossings.values():
        their_laps, their_last = laps_and_last(crossings)
        if their_laps > our_laps:
            ahead += 1
        elif their_laps == our_laps and their_laps > 0 \
                and our_last is not None and their_last < our_last:
            ahead += 1
    return ahead + 1


def ros_to_race_time(lap_stamps, lap_times):
    """Build a converter from ``autoware.log`` stamps to race time.

    Both clocks tick at the same rate, so one shared event fixes the offset: the
    first lap we saw completed in the log happened at ``lap_times[0]`` of race
    time. Returns None when the log never completed a lap, which is exactly when
    there is no lap to attribute an event to either.
    """
    if not lap_stamps or not lap_times:
        return None
    offset = lap_stamps[0] - _cumulative(lap_times)[0]
    return lambda ros_ts: ros_ts - offset


def lap_of(race_time, crossings, completed=None):
    """1-based lap number that ``race_time`` falls in.

    Everything after the last *completed* lap collapses onto one number: the lap
    that was still underway when the session ended. An incident that stopped us
    finishing belongs there, not on the last lap we did complete. ``crossings``
    can run past ``completed`` -- AWSIM logs a line crossing it then declines to
    count, which is exactly the case that must not open a lap 7 in a 6-lap race.
    """
    if completed is None:
        completed = len(crossings)
    for i, end in enumerate(crossings):
        if race_time <= end:
            return min(i + 1, completed + 1)
    return completed + 1


def build(lap_times, penalty_events, recovery_stamps, guard_slow_stamps,
          lap_stamps, ego_crossings=None, npc_crossings=None):
    """One row per lap, plus a row for the unfinished lap if anything happened on it.

    ``penalty_events`` are AWSIM's, already carrying their own lap number and race
    time. ``recovery_stamps`` and ``guard_slow_stamps`` are ROS stamps from
    autoware.log and get converted. Positions come from the NPC crossings when a
    Player.log was kept, and are None otherwise.
    """
    crossings = ego_crossings or _cumulative(lap_times)
    npc_crossings = npc_crossings or {}
    to_race = ros_to_race_time(lap_stamps, lap_times)
    completed = len(lap_times)

    rows = {}

    def row(lap):
        if lap not in rows:
            rows[lap] = {'lap': lap, 'time': None, 'crash': 0, 'wall': 0, 'over': 0,
                         'recoveries': 0, 'guard_slow': 0, 'penalty_s': 0.0,
                         'limit_ranks': []}
        return rows[lap]

    for i, t in enumerate(lap_times):
        row(i + 1)['time'] = round(float(t), 1)

    # A run that completed nothing is the most important kind of result to be able
    # to look at, so give it the one row it is entitled to: the lap it never
    # finished. Without this it has no rows at all and vanishes from the page.
    if not completed:
        row(1)

    for ev in penalty_events or []:
        kind = ev.get('kind')
        if kind not in PENALTY_KINDS:
            continue
        race_time = float(ev.get('race_time') or 0.0)
        # Trust AWSIM's own lap number; fall back to the time when it is absent.
        lap = min(int(ev.get('lap') or 0) or lap_of(race_time, crossings, completed),
                  completed + 1)
        r = row(lap)
        r[kind] += 1
        r['penalty_s'] += float(ev.get('duration') or 0.0)
        if kind == 'over':
            rank = rank_at(race_time, crossings, npc_crossings)
            if rank is not None:
                r['limit_ranks'].append(rank)

    for stamps, field in ((recovery_stamps, 'recoveries'), (guard_slow_stamps, 'guard_slow')):
        if not to_race:
            # No completed lap means no shared event to align the two clocks on --
            # but it also means there is only one lap these can belong to, so they
            # are placed rather than dropped. (A 0-lap run held at a standstill by
            # the guard is *all* guard events; reporting none would be absurd.)
            if not completed and stamps:
                row(1)[field] += len(stamps)
            continue
        for ts in stamps or []:
            race_time = to_race(float(ts))
            r = row(lap_of(race_time, crossings, completed))
            r[field] += 1
            if field == 'guard_slow':
                rank = rank_at(race_time, crossings, npc_crossings)
                if rank is not None:
                    r['limit_ranks'].append(rank)

    out = []
    for lap in sorted(rows):
        r = rows[lap]
        r['penalty_s'] = round(r['penalty_s'], 1)
        ranks = r.pop('limit_ranks')
        # One number for the column: the best position we held while being
        # limited. Several events on one lap are usually the same battle.
        r['limit_rank'] = min(ranks) if ranks else None
        out.append(r)
    return out


def blocker(finished, completed, required, rows, timed_out):
    """Why the run did not finish, in one phrase, from what the data shows.

    Deliberately mechanical: it names the largest observable cost rather than
    guessing at intent. Override it per run with ``"blocker"`` in run_meta.json
    when the real reason is something only a human watched happen.
    """
    if finished:
        return None
    if completed == 0:
        return 'never completed a lap'

    recoveries = sum(r['recoveries'] for r in rows)
    penalty_s = sum(r['penalty_s'] for r in rows)
    crashes = sum(r['crash'] for r in rows)
    walls = sum(r['wall'] for r in rows)

    lap_times = [r['time'] for r in rows if r['time']]
    pace = min(lap_times) if lap_times else None
    remaining = max(required - completed, 0)

    reasons = []
    if penalty_s > 0:
        reasons.append('%.0f s of penalties (%d kart, %d wall)' % (penalty_s, crashes, walls))
    if recoveries:
        reasons.append('%d recoveries' % recoveries)
    if pace and remaining and timed_out:
        reasons.append('%d lap%s still to run at %.0f s best pace'
                       % (remaining, '' if remaining == 1 else 's', pace))
    if not reasons:
        return 'ran out of time'
    return 'out of time: ' + ', '.join(reasons)
