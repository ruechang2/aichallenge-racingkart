#!/usr/bin/env python3
"""Benchmark the cost of the V2X obstacle-avoidance corridor computation.

With ``use_obstacle_avoidance`` the MPC calls
``ReferencePath.update_path_constraints()`` on *every* control cycle, and that
call rasterises obstacles, ray-casts free segments across the corridor and runs
a combinatorial search over which gap to take. At 40 Hz the whole control loop
has 25 ms, so this has to be measured before the feature can be trusted.

Usage (inside the autoware container):
  python3 bench_avoidance.py [--karts 2] [--iters 50]
"""

import argparse
import time

import numpy as np

from multi_purpose_mpc_ros.core.map import Obstacle
from multi_purpose_mpc_ros.tools.reference_path_generator import ReferencePathGenerator


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="/aichallenge/workspace/src/aichallenge_submit/"
                                        "multi_purpose_mpc_ros/config/config.yaml")
    ap.add_argument("--karts", type=int, default=2, help="other karts on track")
    ap.add_argument("--horizon", type=int, default=20, help="MPC N")
    ap.add_argument("--iters", type=int, default=50)
    ap.add_argument("--radius", type=float, default=0.5, help="v2x vehicle_radius")
    ap.add_argument("--width", type=float, default=1.70)
    ap.add_argument("--margin", type=float, default=0.3)
    args = ap.parse_args()

    ref = ReferencePathGenerator.get_reference_path(args.config)
    n_wp = ref.n_waypoints
    print("waypoints: %d  horizon: %d  karts: %d" % (n_wp, args.horizon, args.karts))

    # One obstacle per horizon sample per kart, which is what _v2x_callback
    # produces (a constant-velocity prediction sampled over the horizon).
    def make_obstacles(ego_wp):
        obs = []
        for k in range(args.karts):
            for n in range(args.horizon):
                wp = ref.get_waypoint(ego_wp + 8 + 6 * k + n)
                # sit the kart slightly off the racing line, alternating sides
                off = 0.6 if k % 2 == 0 else -0.6
                obs.append(Obstacle(cx=wp.x + off * np.cos(wp.psi + np.pi / 2),
                                    cy=wp.y + off * np.sin(wp.psi + np.pi / 2),
                                    radius=args.radius))
        return obs

    def bench(with_obstacles):
        times = []
        for i in range(args.iters):
            wp_id = (i * 7) % n_wp
            ref.map.reset_map()
            if with_obstacles:
                ref.map.add_obstacles(make_obstacles(wp_id))
            ref.reset_dynamic_constraints()
            wp = ref.get_waypoint(wp_id)
            t0 = time.perf_counter()
            ref.update_path_constraints(wp_id + 1, [wp.x, wp.y, wp.psi],
                                        args.horizon, 1.087, args.width, args.margin)
            times.append((time.perf_counter() - t0) * 1e3)
        return np.array(times)

    for label, flag in (("clear track", False), ("%d karts ahead" % args.karts, True)):
        t = bench(flag)
        budget = 1000.0 / 40.0
        print("%-16s mean %7.2f ms  p50 %7.2f  p95 %7.2f  max %7.2f   (40 Hz budget %.1f ms) %s"
              % (label, t.mean(), np.percentile(t, 50), np.percentile(t, 95), t.max(),
                 budget, "OK" if np.percentile(t, 95) < budget else "OVER BUDGET"))


if __name__ == "__main__":
    main()
