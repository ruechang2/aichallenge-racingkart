"""race_log の CSV から、自車のラップと停止区間を出す（ホスト側で実行）。
ラップは /awsim/status の自車行（lap 列）で数える。位置があれば走行距離も出す。

    python3 laps.py <csv>
"""
import csv, sys, math
import numpy as np
path = sys.argv[1]
rows = list(csv.DictReader(open(path)))
if not rows:
    print(f"{path}: 空"); sys.exit(0)
t = np.array([float(r['epoch']) for r in rows]); t -= t[0]
v = np.array([float(r['speed']) for r in rows])
lap = np.array([int(float(r.get('lap', -1))) for r in rows])
laptime = np.array([float(r.get('laptime', -1)) for r in rows])
x = np.array([float(r.get('x', 'nan')) for r in rows]); y = np.array([float(r.get('y', 'nan')) for r in rows])
ok = np.isfinite(x) & np.isfinite(y)
dist = float(np.sum(np.hypot(np.diff(x[ok]), np.diff(y[ok])))) if ok.sum() > 1 else float("nan")
print(f"{path}: {t[-1]:.0f}s dist={dist:.0f}m vmean={v.mean():.2f} vmax={v.max():.2f} stopped(<0.5)={100*np.mean(v<0.5):.0f}%")
start = next((t[i] for i in range(len(v)) if v[i] > 1.0), None)
print(f"  発進 t={start}")
for i in range(1, len(lap)):
    if lap[i] != lap[i-1] and lap[i-1] >= 1:
        print(f"  Lap {lap[i-1]} 完了 t={t[i]:6.1f}s  lap time={laptime[i-1]:6.1f}s")
if lap[-1] >= 1:
    print(f"  終了時: Lap {lap[-1]} の {laptime[-1]:.1f}s 経過, sector={rows[-1].get('sector')}")
stopped = v < 0.5; s = None
for i, st in enumerate(stopped):
    if st and s is None: s = i
    if (not st or i == len(stopped)-1) and s is not None:
        if t[i]-t[s] > 3: print(f"  停止 t={t[s]:6.1f}s から {t[i]-t[s]:5.1f}s  lap={lap[s]} sector={rows[s].get('sector')}")
        s = None
