#!/usr/bin/env python3
"""Analyze lane detection benchmark results.

Reads CSV files from the results directory and prints a comparison table.

Usage:
    python3 scripts/analyze_lane_benchmark.py [results_dir]
    Default: /tmp/lane_benchmark
"""

import csv
import os
import sys
from collections import defaultdict


def load_results(results_dir):
    """Load all CSVs, group by detector name."""
    detectors = defaultdict(list)
    for fname in sorted(os.listdir(results_dir)):
        if not fname.endswith('.csv'):
            continue
        # Filename format: {detector}_{timestamp}.csv
        parts = fname.rsplit('_', 2)
        detector = parts[0] if len(parts) >= 3 else fname.replace('.csv', '')

        filepath = os.path.join(results_dir, fname)
        with open(filepath) as f:
            reader = csv.DictReader(f)
            rows = list(reader)
        if rows:
            detectors[detector].append((fname, rows))
    return detectors


def summarize(rows):
    """Compute summary stats from a list of frame rows."""
    if not rows:
        return {}

    # Skip first 10 frames (startup)
    rows = rows[10:] if len(rows) > 10 else rows
    if not rows:
        return {}

    n = len(rows)
    mean_nn = sum(float(r['mean_nn_dist']) for r in rows) / n
    coverage = sum(float(r['coverage']) for r in rows) / n
    fp_rate = sum(float(r['false_positive_rate']) for r in rows) / n
    precision = sum(float(r['precision']) for r in rows) / n
    f1 = sum(float(r['f1']) for r in rows) / n
    point_ratio = sum(float(r['point_ratio']) for r in rows) / n

    # Best/worst frames
    best_f1 = max(float(r['f1']) for r in rows)
    worst_f1 = min(float(r['f1']) for r in rows)

    return {
        'frames': n,
        'mean_nn': mean_nn,
        'coverage': coverage,
        'fp_rate': fp_rate,
        'precision': precision,
        'f1': f1,
        'best_f1': best_f1,
        'worst_f1': worst_f1,
        'point_ratio': point_ratio,
    }


def print_table(detectors):
    if not detectors:
        print("No results found.")
        return

    # For each detector, use the latest run
    summaries = []
    for name, runs in sorted(detectors.items()):
        fname, rows = runs[-1]  # latest run
        s = summarize(rows)
        if s:
            s['name'] = name
            summaries.append(s)

    if not summaries:
        print("No valid results to summarize.")
        return

    # Sort by F1 descending
    summaries.sort(key=lambda s: s['f1'], reverse=True)

    # Header
    cols = [
        ('method', 20),
        ('frames', 6),
        ('mean_nn', 8),
        ('coverage', 9),
        ('precision', 9),
        ('F1', 7),
        ('best_F1', 7),
        ('worst_F1', 8),
        ('pt_ratio', 8),
    ]
    header = ' | '.join(name.ljust(w) for name, w in cols)
    sep = '-+-'.join('-' * w for _, w in cols)
    print(header)
    print(sep)

    for s in summaries:
        row = [
            s['name'].ljust(20),
            str(s['frames']).ljust(6),
            f"{s['mean_nn']:.4f}".ljust(8),
            f"{s['coverage']:.1%}".ljust(9),
            f"{s['precision']:.1%}".ljust(9),
            f"{s['f1']:.3f}".ljust(7),
            f"{s['best_f1']:.3f}".ljust(7),
            f"{s['worst_f1']:.3f}".ljust(8),
            f"{s['point_ratio']:.2f}".ljust(8),
        ]
        print(' | '.join(row))

    # CSV output
    print("\n--- CSV ---")
    print("method,frames,mean_nn,coverage,precision,f1,best_f1,worst_f1,point_ratio")
    for s in summaries:
        print(f"{s['name']},{s['frames']},{s['mean_nn']:.4f},"
              f"{s['coverage']:.4f},{s['precision']:.4f},{s['f1']:.4f},"
              f"{s['best_f1']:.4f},{s['worst_f1']:.4f},{s['point_ratio']:.3f}")


def main():
    results_dir = sys.argv[1] if len(sys.argv) > 1 else '/tmp/lane_benchmark'
    if not os.path.isdir(results_dir):
        print(f"Results directory not found: {results_dir}")
        sys.exit(1)

    detectors = load_results(results_dir)
    print(f"Found results for {len(detectors)} detectors in {results_dir}\n")
    print_table(detectors)


if __name__ == '__main__':
    main()
