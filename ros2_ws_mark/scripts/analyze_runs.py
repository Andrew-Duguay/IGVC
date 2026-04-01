#!/usr/bin/env python3
"""Analyze IGVC obstacle navigation A/B test CSV data.

Reads CSV files from /tmp/igvc_data/, computes per-run metrics,
and prints a comparison table.

Usage:
    python3 scripts/analyze_runs.py [/path/to/igvc_data]
"""

import csv
import math
import os
import sys
from collections import defaultdict

# Track geometry: bottom straight y≈0, top straight y≈10
# Start line: x ≈ -4 (spawn point)
START_LINE_X = -4.0
START_LINE_Y_MIN = -2.0
START_LINE_Y_MAX = 2.0

# SDF obstacle positions for zone detection
SDF_OBSTACLES = [
    (5.5, 0.5, 0.15),
    (-5.5, -0.3, 0.30),
    (-3.0, 10.5, 0.15),
    (2.0, 9.5, 0.15),
    (5.0, 10.0, 0.30),
    (8.5, 3.0, 0.15),
    (-8.5, 7.0, 0.30),
]

OBSTACLE_ZONE_RADIUS = 3.0  # meters from obstacle center


def load_csv(path):
    rows = []
    with open(path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            parsed = {}
            for k, v in row.items():
                try:
                    parsed[k] = float(v)
                except ValueError:
                    parsed[k] = v
            rows.append(parsed)
    return rows


def near_obstacle(x, y):
    for ox, oy, _ in SDF_OBSTACLES:
        if math.sqrt((x - ox) ** 2 + (y - oy) ** 2) < OBSTACLE_ZONE_RADIUS:
            return True
    return False


def detect_lap_crossings(rows):
    """Detect start-line crossings (x crossing START_LINE_X going positive)."""
    crossings = []
    for i in range(1, len(rows)):
        x0 = rows[i - 1]['world_x']
        x1 = rows[i]['world_x']
        y = rows[i]['world_y']
        if (START_LINE_Y_MIN < y < START_LINE_Y_MAX and
                x0 < START_LINE_X <= x1):
            crossings.append(rows[i]['timestamp'])
    return crossings


def compute_metrics(rows):
    if not rows:
        return None

    label = rows[0].get('approach_label', 'unknown')
    duration = rows[-1]['timestamp'] - rows[0]['timestamp']

    # Lap times
    crossings = detect_lap_crossings(rows)
    lap_times = []
    for i in range(1, len(crossings)):
        lap_times.append(crossings[i] - crossings[i - 1])

    # Steering variance
    az_vals = [r['cmd_az'] for r in rows]
    az_mean = sum(az_vals) / len(az_vals) if az_vals else 0
    az_var = sum((a - az_mean) ** 2 for a in az_vals) / len(az_vals) if az_vals else 0

    # Min clearance
    min_clearance = min(r['min_obstacle_clearance'] for r in rows)

    # Stop count (transitions to stopped)
    stops = 0
    for i in range(1, len(rows)):
        if rows[i]['is_stopped'] == 1 and rows[i - 1]['is_stopped'] == 0:
            stops += 1

    # Speed in obstacle zones vs clear zones
    obs_speeds = []
    clear_speeds = []
    for r in rows:
        if near_obstacle(r['world_x'], r['world_y']):
            obs_speeds.append(r['cmd_vx'])
        else:
            clear_speeds.append(r['cmd_vx'])

    avg_speed_obs = sum(obs_speeds) / len(obs_speeds) if obs_speeds else 0
    avg_speed_clear = sum(clear_speeds) / len(clear_speeds) if clear_speeds else 0

    # Path stability (variance of path_y_at_2m)
    py_vals = [r['path_y_at_2m'] for r in rows if r['path_y_at_2m'] != 0]
    py_mean = sum(py_vals) / len(py_vals) if py_vals else 0
    py_var = sum((p - py_mean) ** 2 for p in py_vals) / len(py_vals) if py_vals else 0

    return {
        'label': label,
        'duration': duration,
        'laps': len(crossings) - 1 if len(crossings) > 1 else 0,
        'lap_times': lap_times,
        'avg_lap_time': sum(lap_times) / len(lap_times) if lap_times else float('nan'),
        'steering_var': az_var,
        'min_clearance': min_clearance,
        'stops': stops,
        'avg_speed_obs': avg_speed_obs,
        'avg_speed_clear': avg_speed_clear,
        'path_stability': py_var,
        'total_samples': len(rows),
    }


def print_comparison(all_metrics):
    if not all_metrics:
        print("No data to compare.")
        return

    # Header
    cols = [
        ('Approach', 12),
        ('Laps', 5),
        ('Avg Lap(s)', 10),
        ('Steer Var', 10),
        ('Min Clr(m)', 10),
        ('Stops', 6),
        ('Spd Obs', 8),
        ('Spd Clr', 8),
        ('Path Var', 9),
    ]

    header = '  '.join(f'{name:>{w}}' for name, w in cols)
    print('\n' + '=' * len(header))
    print('IGVC Obstacle Navigation — Comparative Results')
    print('=' * len(header))
    print(header)
    print('-' * len(header))

    for m in all_metrics:
        vals = [
            f"{m['label']:>12}",
            f"{m['laps']:>5d}",
            f"{m['avg_lap_time']:>10.1f}" if not math.isnan(m['avg_lap_time']) else f"{'N/A':>10}",
            f"{m['steering_var']:>10.4f}",
            f"{m['min_clearance']:>10.3f}",
            f"{m['stops']:>6d}",
            f"{m['avg_speed_obs']:>8.3f}",
            f"{m['avg_speed_clear']:>8.3f}",
            f"{m['path_stability']:>9.5f}",
        ]
        print('  '.join(vals))

    print('-' * len(header))

    # Per-lap breakdown
    print('\nPer-Lap Times:')
    for m in all_metrics:
        if m['lap_times']:
            times_str = ', '.join(f'{t:.1f}s' for t in m['lap_times'])
            print(f"  {m['label']:>12}: {times_str}")
        else:
            print(f"  {m['label']:>12}: No complete laps detected")

    print()


def main():
    data_dir = sys.argv[1] if len(sys.argv) > 1 else '/tmp/igvc_data'

    if not os.path.isdir(data_dir):
        print(f"Directory not found: {data_dir}")
        sys.exit(1)

    csv_files = sorted(f for f in os.listdir(data_dir) if f.endswith('.csv'))
    if not csv_files:
        print(f"No CSV files found in {data_dir}")
        sys.exit(1)

    print(f"Found {len(csv_files)} CSV file(s) in {data_dir}:")
    for f in csv_files:
        print(f"  {f}")

    all_metrics = []
    for f in csv_files:
        path = os.path.join(data_dir, f)
        rows = load_csv(path)
        if rows:
            metrics = compute_metrics(rows)
            if metrics:
                metrics['file'] = f
                all_metrics.append(metrics)

    print_comparison(all_metrics)


if __name__ == '__main__':
    main()
