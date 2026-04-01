#!/usr/bin/env python3
"""Analyze NML approach test results.

Reads JSON files from the results directory and prints a CSV summary table.

Usage:
    python3 scripts/analyze_nml_results.py [results_dir]
    Default results_dir: /tmp/nml_results
"""

import json
import os
import sys


def load_results(results_dir):
    results = []
    for fname in sorted(os.listdir(results_dir)):
        if not fname.endswith('.json'):
            continue
        filepath = os.path.join(results_dir, fname)
        with open(filepath, 'r') as f:
            data = json.load(f)
        results.append(data)
    return results


def print_summary(results):
    if not results:
        print("No results found.")
        return

    # Header
    cols = [
        ('approach', 10),
        ('laps', 5),
        ('collisions', 11),
        ('violations', 11),
        ('min_obs_clr', 11),
        ('min_lane_m', 10),
        ('lap1_time', 9),
        ('lap2_time', 9),
        ('stuck', 6),
    ]

    header = ' | '.join(name.ljust(width) for name, width in cols)
    sep = '-+-'.join('-' * width for _, width in cols)
    print(header)
    print(sep)

    for r in results:
        approach = r.get('approach', '?')
        laps = r.get('laps_completed', 0)
        collisions = r.get('total_collisions', 0)
        violations = r.get('total_violations', 0)
        min_obs = r.get('min_obs_clearance')
        min_lane = r.get('min_lane_margin')
        lap_times = r.get('lap_times', [])
        stuck = r.get('stuck_events', 0)

        lap1 = f"{lap_times[0]:.1f}s" if len(lap_times) > 0 else "—"
        lap2 = f"{lap_times[1]:.1f}s" if len(lap_times) > 1 else "—"
        obs_str = f"{min_obs:.3f}" if min_obs is not None else "—"
        lane_str = f"{min_lane:.3f}" if min_lane is not None else "—"

        row = [
            approach.ljust(10),
            str(laps).ljust(5),
            str(collisions).ljust(11),
            str(violations).ljust(11),
            obs_str.ljust(11),
            lane_str.ljust(10),
            lap1.ljust(9),
            lap2.ljust(9),
            str(stuck).ljust(6),
        ]
        print(' | '.join(row))

    # Print collision details
    print("\n--- Collision Details ---")
    for r in results:
        approach = r.get('approach', '?')
        details = r.get('collision_details', [])
        if details:
            for d in details:
                print(f"  {approach}: {d['name']} at ({d['x']:.1f},{d['y']:.1f}) pen={d['penetration']:.3f}m")

    # Print violation breakdown
    print("\n--- Violation Breakdown by Segment ---")
    for r in results:
        approach = r.get('approach', '?')
        segs = r.get('violation_count_by_segment', {})
        if segs:
            seg_str = ', '.join(f"{k}={v}" for k, v in sorted(segs.items()))
            print(f"  {approach}: {seg_str}")

    # CSV output
    print("\n--- CSV ---")
    print("approach,laps,collisions,violations,min_obs_clearance,min_lane_margin,lap1_time,lap2_time,stuck")
    for r in results:
        approach = r.get('approach', '?')
        laps = r.get('laps_completed', 0)
        collisions = r.get('total_collisions', 0)
        violations = r.get('total_violations', 0)
        min_obs = r.get('min_obs_clearance')
        min_lane = r.get('min_lane_margin')
        lap_times = r.get('lap_times', [])
        stuck = r.get('stuck_events', 0)

        lap1 = f"{lap_times[0]:.1f}" if len(lap_times) > 0 else ""
        lap2 = f"{lap_times[1]:.1f}" if len(lap_times) > 1 else ""
        obs_str = f"{min_obs:.3f}" if min_obs is not None else ""
        lane_str = f"{min_lane:.3f}" if min_lane is not None else ""

        print(f"{approach},{laps},{collisions},{violations},{obs_str},{lane_str},{lap1},{lap2},{stuck}")


def main():
    results_dir = sys.argv[1] if len(sys.argv) > 1 else '/tmp/nml_results'
    if not os.path.isdir(results_dir):
        print(f"Results directory not found: {results_dir}")
        sys.exit(1)

    results = load_results(results_dir)
    print(f"Found {len(results)} result files in {results_dir}\n")
    print_summary(results)


if __name__ == '__main__':
    main()
