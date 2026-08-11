#!/usr/bin/env python3
"""Parse AWSIM/Autoware run logs into the dashboard data block.

Scans ``output/<timestamp>/d*/autoware.log``, extracts per-run config, lap times
and guard activity, auto-derives the "change vs previous run", classifies the
result, and injects the JSON into ``dashboard/dashboard.html`` (between the
``id="run-data"> ... </script>`` markers). Pure stdlib, no ROS needed.

Usage:
    python3 dashboard/parse_runs.py              # refresh the dashboard in place
    python3 dashboard/parse_runs.py --print      # also print the JSON
    python3 dashboard/parse_runs.py --target 6   # different lap target

Lap times come from AWSIM's ``result-summary.json`` when one sits next to the log
(save it after a race run — see dashboard/README.md), because in a race session the
controller's own ``Lap N completed`` line reports cumulative time, not per-lap.

Optional per-run overrides: drop a ``run_meta.json`` next to the log
(``output/<ts>/d1/run_meta.json``) with any of:
    {"collisions": 0, "change": "note", "label": "my run", "exclude": false,
     "laps": [64.8, 108.4]}
Collisions are not in autoware.log (they live in the AWSIM container's
Player.log); capture them there and record via run_meta.json if you want them.
"""
import argparse
import glob
import json
import os
import re
import statistics

import lap_detail

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)

RE_CFG = lambda name: re.compile(r'^\[run_mpc[^\]]*\]\s+' + name + r':\s+(-?[\d.]+)', re.M)
RE_FLAG = lambda name: re.compile(r'^\[run_mpc[^\]]*\]\s+' + name + r':\s+(true|false)', re.M | re.I)
RE_LIST = lambda name: re.compile(r'^\[run_mpc[^\]]*\]\s+' + name + r':\s+\[([^\]]*)\]', re.M)
RE_REFVEL = re.compile(r'^\[run_mpc[^\]]*\]\s+ref_vel:\s+([\d.]+)', re.M)
RE_GUARD = re.compile(r'collision_guard up \(v2x=(\w+), scan=(\w+)')
RE_RECOVERY = re.compile(r'stuck_recovery up \(enabled=(\w+)')
# One WARN per incident, so counting them counts recoveries — the whole point of a
# traffic run is how often the car had to dig itself out, not just whether it finished.
RE_STUCK = re.compile(r'STUCK detected')
# Stamped versions of the two events that have to land on the right lap.
RE_STUCK_TS = re.compile(r'\[(\d{10}\.\d+)\] \[stuck_recovery\]: STUCK detected')
RE_SLOW_TS = re.compile(r'\[(\d{10}\.\d+)\] \[collision_guard\]: slow: cap')
# Lateral avoidance is a launch arg, not a config value, so it only shows up as the
# controller's own startup warning.
RE_AVOID = re.compile(r'USE_OBSTACLE_AVOIDANCE is enabled')
# Whether any other kart was actually present. Without this a traffic run and a clear
# run look identical in the table, and their lap times are not comparable.
RE_TRAFFIC = re.compile(r'traffic: kart |obstacle points in corridor')
RE_LAP = re.compile(r'\[(\d{10}\.\d+)\] \[mpc_controller\]:[^\n]*?Lap (\d+) completed! Lap time: ([\d.]+) s')
RE_ANY_TS = re.compile(r'\[(\d{10}\.\d+)\]')
# Limits are also settable at runtime; the node logs every accepted change.
RE_PARAM = re.compile(r'\[mpc_controller\]: (\w+(?:\[\d\])?) was updated to \'([^\']+)\'')

# Commented-out preset blocks in config.yaml are echoed too, but they carry a
# leading '#', which the \s+ in the patterns above will not match — so only the
# live values are picked up.


def _cfg(txt, name):
    m = RE_CFG(name).search(txt)
    return float(m.group(1)) if m else None


def _flag(txt, name):
    m = RE_FLAG(name).search(txt)
    return m.group(1).lower() == 'true' if m else None


def _list0(txt, name):
    """First element of a `name: [a, b, c]` config line."""
    m = RE_LIST(name).search(txt)
    if not m:
        return None
    try:
        return float(m.group(1).split(',')[0])
    except ValueError:
        return None


PARAM_FIELD = {'v_max': 'vmax', 'a_max': 'amax', 'a_min': 'amin', 'ay_max': 'ay',
               'Q[0]': 'q0', 'wp_id_offset': 'wp_off'}


# A lap under this is not physically possible: the raceline is ~345 m and the kart
# tops out at 37 km/h, so even flat out the whole way a lap takes 33.6 s. Values
# below it are simulator artefacts (a spurious line trigger after a wall recovery,
# for instance) and would otherwise become the headline "best lap".
MIN_PLAUSIBLE_LAP_S = 30.0


def _load_json(path):
    try:
        with open(path) as fh:
            return json.load(fh)
    except Exception:
        return None


def awsim_details(logdir):
    """AWSIM's per-vehicle record: laps, whether it finished, and every penalty.

    Written only by the evaluation bundle (``run_evaluation.bash``), so a run
    started as bare ``make simulator-<mode>`` + ``make autoware-simulator`` has
    none and its collision counts are simply unknown — not zero.
    """
    for name in ('d1-result-details.json', 'result-details.json'):
        data = _load_json(os.path.join(logdir, name))
        if data:
            return data
    return None


def awsim_player_log(logdir):
    """Crossing times for us and the NPCs, for reconstructing our position.

    Player.log lives inside the simulator container and is lost with it, so this
    is only present for runs where it was copied out afterwards
    (``tools/save_sim_artifacts.sh``).
    """
    path = os.path.join(logdir, 'Player.log')
    if not os.path.exists(path):
        return None, {}
    with open(path, errors='replace') as fh:
        return lap_detail.parse_player_log(fh.read())


def awsim_summary(logdir):
    """The judge's session record: our position, whether we finished, the target.

    Written by AWSIM itself, so it exists for any run whose result-summary.json
    was kept — including ones started without the evaluation bundle, which have
    no result-details.json and therefore no contact data.
    """
    data = _load_json(os.path.join(logdir, 'result-summary.json'))
    if not data:
        return {}
    required = (data.get('session') or {}).get('required_laps')
    for veh in data.get('vehicles') or []:
        if veh.get('laps'):
            return {'final_position': veh.get('final_position'),
                    'finished': veh.get('finished'),
                    'required_laps': required}
    return {'required_laps': required}


def awsim_lap_times(logdir):
    """Per-lap times straight from AWSIM's ``result-summary.json``, if it is there.

    The controller's own ``Lap N completed`` line is only as good as the number
    AWSIM hands it, and in a race session (NPCs, ranking on) that number is the
    *cumulative* session time, not the lap: a 6-lap race logged
    110/215/275/327/377/425 s against AWSIM's own 107/107/61/50/50/50 s. Where
    the simulator's own record exists it outranks the log.
    """
    details = awsim_details(logdir)
    if details and details.get('laps'):
        return [float(t) for t in details['laps']]
    data = _load_json(os.path.join(logdir, 'result-summary.json'))
    if not data:
        return None
    # Multi-vehicle sessions list every car; ours is the one that drove.
    for veh in data.get('vehicles') or []:
        if veh.get('laps'):
            return [float(t) for t in veh['laps']]
    laps = data.get('laps')
    return [float(t) for t in laps] if laps else None


def parse_log(path, min_lap=MIN_PLAUSIBLE_LAP_S):
    with open(path, errors='replace') as fh:
        txt = fh.read()

    vmax, amax, ay = _cfg(txt, 'v_max'), _cfg(txt, 'a_max'), _cfg(txt, 'ay_max')
    amin, width = _cfg(txt, 'a_min'), _cfg(txt, 'width')
    margin = _cfg(txt, 'safety_margin')
    # Also separates AWSIM eras: the 2026-08-03 build dropped the vehicle's steer rate
    # limit 0.8 -> 0.6, so runs either side of it are not comparable pace.
    steer = _cfg(txt, 'steer_rate_max')
    q0 = _list0(txt, 'Q')
    profile = _flag(txt, 'use_speed_profile')
    wp_off = _cfg(txt, 'wp_id_offset')
    avoid = bool(RE_AVOID.search(txt))
    traffic = bool(RE_TRAFFIC.search(txt))

    refs = [float(x) for x in RE_REFVEL.findall(txt)]
    # ref_vel sections print in file order: s1,s1_1,s2,s3,s4,s5,s6,s7,s8,s9
    # the manually-capped corners are s4,s6,s8 -> indices 4,6,8
    corners = None
    if len(refs) >= 9:
        corners = '/'.join(str(int(round(refs[i]))) for i in (4, 6, 8))

    gm = RE_GUARD.search(txt)
    if gm:
        guard = ('V2X on' if gm.group(1) == 'True' else 'V2X off') + ' · ' + \
                ('scan on' if gm.group(2) == 'True' else 'scan off')
    else:
        guard = '—'

    emerg = len(re.findall(r'EMERGENCY BRAKE', txt))
    slow = len(re.findall(r'slow: cap', txt))

    rm = RE_RECOVERY.search(txt)
    recovery = (rm.group(1).lower() == 'true') if rm else None
    recoveries = len(RE_STUCK.findall(txt))

    laps = sorted((int(n), float(t), float(ts)) for ts, n, t in RE_LAP.findall(txt))
    # AWSIM's own per-lap record wins over the controller's log where both exist.
    authoritative = awsim_lap_times(os.path.dirname(path))
    if authoritative:
        stamps = [ts for _, _, ts in laps]
        laps = [(i + 1, t, stamps[i] if i < len(stamps) else None)
                for i, t in enumerate(authoritative)]
        laps = [(n, t, ts) for n, t, ts in laps if ts is not None]
    dropped = [t for _, t, _ in laps if t < min_lap]
    laps = [(n, t, ts) for n, t, ts in laps if t >= min_lap]
    lap_times = [round(t, 1) for _, t, _ in laps]
    last_lap_ts = laps[-1][2] if laps else None

    all_ts = RE_ANY_TS.findall(txt)
    last_ts = float(all_ts[-1]) if all_ts else None
    first_ts = float(all_ts[0]) if all_ts else None

    # Live `ros2 param set` changes, in the order the node actually applied them.
    events = []
    for line in txt.splitlines():
        pm = RE_PARAM.search(line)
        if not pm:
            continue
        field = PARAM_FIELD.get(pm.group(1))
        if field is None:
            continue
        tsm = RE_ANY_TS.search(line)
        try:
            value = float(pm.group(2))
        except ValueError:
            continue
        if tsm:
            events.append((float(tsm.group(1)), field, value))

    # --- per-lap breakdown ---
    logdir = os.path.dirname(path)
    details = awsim_details(logdir) or {}
    ego_crossings, npc_crossings = awsim_player_log(logdir)
    lapdetail = lap_detail.build(
        lap_times=[t for _, t, _ in laps],
        penalty_events=details.get('penalty_events'),
        recovery_stamps=[float(t) for t in RE_STUCK_TS.findall(txt)],
        guard_slow_stamps=[float(t) for t in RE_SLOW_TS.findall(txt)],
        lap_stamps=[ts for _, _, ts in laps],
        ego_crossings=ego_crossings,
        npc_crossings=npc_crossings,
    )
    summary = awsim_summary(logdir)
    required = int(details.get('required_laps') or summary.get('required_laps') or 0) or None
    finished = details.get('finished')
    if finished is None:
        finished = summary.get('finished')
    # Only the judge knows whether it finished; without its file, say so rather
    # than inferring "no" from a lap count that may just be a short dev session.
    timed_out = bool(details.get('session_timeout')) and finished is False
    blocked_by = None
    if finished is not None:
        blocked_by = lap_detail.blocker(finished, len(lap_times),
                                        required or 0, lapdetail, timed_out)

    return dict(vmax=vmax, amax=amax, amin=amin, ay=ay, q0=q0, width=width,
                margin=margin, steer=steer, avoid=avoid, traffic=traffic,
                profile=profile, wp_off=wp_off, corners=corners, guard=guard,
                recovery=recovery, recoveries=recoveries,
                lapdetail=lapdetail, blocked_by=blocked_by, finished=finished,
                required=required, has_details=bool(details),
                penalty_s=round(float(details.get('penalty_total_seconds') or 0.0), 1),
                final_position=summary.get('final_position'),
                emerg=emerg, slow=slow, lap_times=lap_times, laps=laps,
                events=events, last_lap_ts=last_lap_ts, last_ts=last_ts,
                first_ts=first_ts, dropped_laps=dropped, has_cfg=vmax is not None)


def classify(rec, target):
    n = len(rec['lap_times'])
    if n >= target:
        return 'ok'
    if n == 0:
        return 'partial'
    # completed < target: stall heuristic — did the sim keep running long after
    # the last completed lap without finishing another?
    if rec['last_ts'] and rec['last_lap_ts']:
        med = statistics.median(rec['lap_times'])
        if (rec['last_ts'] - rec['last_lap_ts']) > 1.4 * med:
            return 'fail'
    return 'partial'


def fly_best(lap_times):
    fly = lap_times[1:] or lap_times
    return min(fly) if fly else None


def load_meta(logdir):
    for cand in (os.path.join(logdir, 'run_meta.json'),
                 os.path.join(os.path.dirname(logdir), 'run_meta.json')):
        if os.path.exists(cand):
            try:
                with open(cand) as fh:
                    return json.load(fh)
            except Exception:
                pass
    return {}


DIFF_FIELDS = [('v_max', 'vmax'), ('ay_max', 'ay'), ('a_max', 'amax'), ('a_min', 'amin'),
               ('Q[0]', 'q0'), ('width', 'width'), ('safety_margin', 'margin'),
               ('steer_rate_max', 'steer'),
               ('profile', 'profile'), ('avoidance', 'avoid'), ('traffic', 'traffic'),
               ('wp_id_offset', 'wp_off'), ('ref_vel', 'corners'), ('guard', 'guard'),
               ('recovery', 'recovery')]


def _fmt(v):
    if isinstance(v, bool):
        return 'on' if v else 'off'
    if isinstance(v, float):
        return ('%g' % v)
    return str(v)


def diff_change(cur, prev):
    if prev is None:
        return 'baseline'
    parts = []
    for label, key in DIFF_FIELDS:
        a, b = prev.get(key), cur.get(key)
        if a != b and b is not None:
            parts.append('%s %s→%s' % (label, _fmt(a), _fmt(b)))
    return ', '.join(parts) if parts else 'repeat'


def split_segments(rec):
    """Split one log into config segments at live `ros2 param set` boundaries.

    A long run where ay_max was walked 10 → 13 → 16 → 19 is four experiments, not
    one; attributing all of its laps to the startup config would hide exactly the
    comparison this dashboard exists to make. Laps are assigned by completion
    time, so the first lap of each segment straddles the change — it is treated
    like a standing-start lap and excluded from the segment's best.
    """
    base = {k: rec.get(k) for _, k in DIFF_FIELDS}
    laps = rec.get('laps') or []
    events = rec.get('events') or []
    if not events:
        return [(base, rec['lap_times'], False)]

    bounds = [0.0] + [ts for ts, _, _ in events] + [float('inf')]
    cfgs, cur = [], dict(base)
    cfgs.append(dict(cur))
    for _, field, value in events:
        cur[field] = value
        cfgs.append(dict(cur))

    segments = []
    for i, cfg in enumerate(cfgs):
        lo, hi = bounds[i], bounds[i + 1]
        times = [round(t, 1) for _, t, ts in laps if lo <= ts < hi]
        if times:
            segments.append((cfg, times, i > 0))
    return segments or [(base, rec['lap_times'], False)]


def collect(output_dir, target, include_all, min_laps=1, since=None):
    logs = {}
    # prefer d1 (vehicle domain), fall back to any d*
    for path in glob.glob(os.path.join(output_dir, '*', 'd*', 'autoware.log')):
        if os.path.islink(path):
            continue
        run_ts = os.path.basename(os.path.dirname(os.path.dirname(path)))
        dom = os.path.basename(os.path.dirname(path))
        prev = logs.get(run_ts)
        if prev is None or (dom == 'd1' and prev[0] != 'd1'):
            logs[run_ts] = (dom, path)

    runs = []
    for run_ts in sorted(logs):
        dom, path = logs[run_ts]
        rec = parse_log(path)
        meta = load_meta(os.path.dirname(path))
        if meta.get('exclude'):
            continue
        # Last resort for a run whose log holds cumulative times and whose
        # result-summary.json was not kept: state the real per-lap times by hand
        # rather than leave impossible ones (a 503 s lap in a 600 s session) in
        # the table. Timestamps stay as logged, so segmenting still works.
        if meta.get('laps'):
            override = [float(t) for t in meta['laps']]
            rec = dict(rec,
                       lap_times=[round(t, 1) for t in override],
                       laps=[(n, t, ts) for (n, _, ts), t in zip(rec['laps'], override)])
        n = len(rec['lap_times'])
        # keep:true overrides both length gates, including the 0-lap one: a run that
        # never completed a lap is the most important kind of 5-lap-completion result,
        # so an explicitly kept one must not be silently dropped as a boot-only stub.
        keep = bool(meta.get('keep'))
        if n == 0 and not include_all and not keep:
            continue  # boot-only / failed-to-drive run
        if n < min_laps and not keep:
            continue  # interrupted / too-short run (override with run_meta keep:true)
        # `since` is applied at the end, not here: run ids have to stay put. R58 is
        # written into run_meta notes and two READMEs, and would silently become R16
        # the moment the window moved if numbering only counted what is displayed.
        shown = not (since and run_ts[:8] < since)
        runs.append((run_ts, rec, meta, shown))

    # Order by when the run actually happened. Directory names are usually
    # timestamps, but not always (e.g. `20260726-w170`), and a wrong order would
    # invert every "change vs previous" diff — so the log's own first ROS stamp wins.
    runs.sort(key=lambda r: (r[1]['first_ts'] is None, r[1]['first_ts'], r[0]))

    out, prev_cfg = [], None
    n_out = 0
    for run_ts, rec, meta, shown in runs:
        # `LOG_DIR` is usually a timestamp, but a hand-named directory
        # (`20260726-w170`) must not be sliced into nonsense like "w1:70".
        nice_time = '%s-%s %s:%s' % (run_ts[4:6], run_ts[6:8], run_ts[9:11], run_ts[11:13]) \
            if re.fullmatch(r'\d{8}-\d{6}', run_ts) else run_ts
        segments = split_segments(rec)
        for seg_i, (cfg, lap_times, is_live) in enumerate(segments):
            n_out += 1
            label = meta.get('label') or nice_time
            if len(segments) > 1:
                label += ' · %s' % chr(ord('a') + seg_i)
            change = diff_change(cfg, prev_cfg)
            if seg_i == 0 and meta.get('change'):
                change = meta['change']
            elif is_live:
                change += ' (live)'
            # The stall heuristic only makes sense for the segment the log ends
            # on; an earlier segment ended because a param changed, not a stall.
            seg_rec = dict(rec, lap_times=lap_times)
            if seg_i < len(segments) - 1:
                seg_rec['last_ts'] = seg_rec['last_lap_ts'] = None
            if not shown:
                prev_cfg = cfg   # still the baseline the next visible run diffs against
                continue
            out.append({
                'id': 'R%d' % n_out,
                'time': label,
                'change': change,
                'vmax': cfg['vmax'], 'amax': cfg['amax'], 'amin': cfg['amin'],
                'ay': cfg['ay'], 'q0': cfg['q0'], 'width': cfg['width'],
                'margin': cfg['margin'], 'steer': cfg['steer'], 'avoid': cfg['avoid'],
                'traffic': cfg['traffic'],
                'profile': cfg['profile'], 'wp_off': cfg['wp_off'],
                'corners': cfg['corners'] or '—', 'guard': cfg['guard'],
                'recovery': cfg['recovery'], 'recoveries': rec['recoveries'],
                # Per-lap breakdown, and why the run stopped short. Only the
                # evaluation bundle writes the file these come from, so
                # has_details=false means "not recorded", never "none happened".
                'lapdetail': rec['lapdetail'] if seg_i == 0 else [],
                'has_details': rec['has_details'],
                'finished': rec['finished'],
                'required': rec['required'],
                'penalty_s': rec['penalty_s'],
                'final_position': rec['final_position'],
                'blocked_by': meta.get('blocker') or rec['blocked_by'],
                'laps': lap_times, 'completed': len(lap_times),
                # The session's own requirement beats the dashboard's default: a
                # 6-lap race that managed 5 is not "5/5 done".
                'target': rec['required'] or target,
                'collisions': meta.get('collisions'),
                'guard_emergency': rec['emerg'], 'guard_slow': rec['slow'],
                'result': meta.get('result') or classify(seg_rec, rec['required'] or target),
                'note': meta.get('note'),
                'live': is_live,
            })
            prev_cfg = cfg

    # tag the globally fastest flying lap as "best"
    bests = [(fly_best(r['laps']), r) for r in out if r['laps']]
    bests = [(b, r) for b, r in bests if b is not None]
    if bests:
        _, champ = min(bests, key=lambda x: x[0])
        if champ['result'] == 'ok':
            champ['result'] = 'best'
    return out


def build_payload(runs, target, target_s):
    from datetime import datetime
    cur = None
    if runs:
        last = runs[-1]
        cur = {k: last[k] for k in ('vmax', 'amax', 'amin', 'ay', 'q0', 'width',
                                    'margin', 'steer', 'avoid', 'traffic',
                                    'profile', 'corners', 'guard', 'recovery')}
    return {
        'generated': datetime.now().strftime('%Y-%m-%d %H:%M'),
        'target_laps': target,
        'target_s': target_s,
        'current': cur,
        'runs': runs,
    }


def inject(html_path, payload):
    with open(html_path, encoding='utf-8') as fh:
        html = fh.read()
    block = json.dumps(payload, ensure_ascii=False, indent=2)
    pat = re.compile(r'(<script type="application/json" id="run-data">)(.*?)(</script>)', re.S)
    if not pat.search(html):
        raise SystemExit('run-data block not found in %s' % html_path)
    html2 = pat.sub(lambda m: m.group(1) + '\n' + block + '\n' + m.group(3), html)
    with open(html_path, 'w', encoding='utf-8') as fh:
        fh.write(html2)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--output-dir', default=os.path.join(REPO, 'output'),
                    help='directory holding <timestamp>/d*/autoware.log (default: repo output/)')
    ap.add_argument('--html', default=os.path.join(HERE, 'dashboard.html'),
                    help='dashboard HTML to update in place')
    ap.add_argument('--target', type=int, default=5, help='lap target (default 5)')
    ap.add_argument('--target-s', type=float, default=40.0,
                    help='per-lap time target in seconds (default 40)')
    ap.add_argument('--min-laps', type=int, default=1,
                    help='drop runs with fewer completed laps (default 1; run_meta keep:true overrides)')
    # Runs before August are from older AWSIM builds (the 2026-08-03 one dropped the
    # steer rate limit 0.8 -> 0.6, so their pace is not comparable) and none of them
    # carry the per-lap contact data this page is built around. Their logs are still
    # in output/ -- pass --since '' to bring them back.
    ap.add_argument('--since', default='20260801', metavar='YYYYMMDD',
                    help="drop runs before this date (default 20260801; pass '' for all)")
    ap.add_argument('--all', action='store_true', help='include runs that completed 0 laps')
    ap.add_argument('--print', dest='show', action='store_true', help='print the JSON payload')
    args = ap.parse_args()

    runs = collect(args.output_dir, args.target, args.all,
                   min_laps=args.min_laps, since=args.since)
    payload = build_payload(runs, args.target, args.target_s)

    inject(args.html, payload)
    json_path = os.path.join(HERE, 'run_data.json')
    with open(json_path, 'w', encoding='utf-8') as fh:
        json.dump(payload, fh, ensure_ascii=False, indent=2)

    print('Parsed %d run(s) from %s' % (len(runs), args.output_dir))
    for r in runs:
        b = fly_best(r['laps'])
        print('  %-3s %-30s best=%s laps=%d/%d %s' % (
            r['id'], (r['change'] or '')[:30],
            ('%.1f' % b) if b is not None else '—',
            r['completed'], r['target'], r['result']))
    print('Updated %s' % args.html)
    print('Wrote   %s' % json_path)
    if args.show:
        print(json.dumps(payload, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
