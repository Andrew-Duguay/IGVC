#!/usr/bin/env python3
"""Lane detection benchmark scoring node.

Subscribes to ground-truth (/benchmark/lane_points) and candidate (/candidate/lane_points),
uses message_filters to synchronize them by timestamp, then computes accuracy metrics.

Metrics:
- mean_nn_dist: Mean nearest-neighbor distance (candidate -> ground truth)
- coverage: % of ground truth points matched within match_radius
- false_positive_rate: % of candidate points far from any ground truth
- f1: harmonic mean of coverage and precision
"""

import csv
import math
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from message_filters import ApproximateTimeSynchronizer, Subscriber


class LaneBenchmarkNode(Node):
    def __init__(self):
        super().__init__('lane_benchmark_node')

        self.declare_parameter('match_radius', 0.3)
        self.declare_parameter('fp_radius', 0.5)
        self.declare_parameter('results_dir', '/tmp/lane_benchmark')
        self.declare_parameter('detector_name', 'unknown')

        self._match_r = self.get_parameter('match_radius').value
        self._fp_r = self.get_parameter('fp_radius').value
        self._detector = self.get_parameter('detector_name').value

        self._frame_count = 0

        # Results file
        results_dir = self.get_parameter('results_dir').value
        os.makedirs(results_dir, exist_ok=True)
        ts = time.strftime('%Y%m%d_%H%M%S')
        self._csv_path = os.path.join(results_dir, f'{self._detector}_{ts}.csv')
        self._csv_file = open(self._csv_path, 'w', newline='')
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow([
            'frame', 'time', 'gt_count', 'cand_count', 'point_ratio',
            'mean_nn_dist', 'median_nn_dist', 'max_nn_dist',
            'coverage', 'false_positive_rate', 'precision', 'f1',
        ])

        # Synchronized subscribers
        gt_sub = Subscriber(self, PointCloud2, '/benchmark/lane_points')
        cand_sub = Subscriber(self, PointCloud2, '/candidate/lane_points')
        self.sync = ApproximateTimeSynchronizer(
            [gt_sub, cand_sub], queue_size=20, slop=0.5)
        self.sync.registerCallback(self._sync_cb)

        self.get_logger().info(
            f'Lane benchmark node started (detector={self._detector}, synced)')

    def _sync_cb(self, gt_msg, cand_msg):
        gt = [(p[0], p[1]) for p in
              point_cloud2.read_points(gt_msg, field_names=('x', 'y'), skip_nans=True)]
        cand = [(p[0], p[1]) for p in
                point_cloud2.read_points(cand_msg, field_names=('x', 'y'), skip_nans=True)]

        if not gt or not cand:
            return

        self._frame_count += 1
        match_r = self._match_r
        fp_r = self._fp_r

        # Convert to numpy for faster computation
        gt_arr = np.array(gt)
        cand_arr = np.array(cand)

        # Nearest-neighbor: candidate -> ground truth (vectorized)
        nn_dists = np.empty(len(cand_arr))
        for i, (cx, cy) in enumerate(cand_arr):
            dists = np.sqrt((gt_arr[:, 0] - cx)**2 + (gt_arr[:, 1] - cy)**2)
            nn_dists[i] = dists.min()

        fp_count = np.sum(nn_dists > fp_r)

        # Coverage: ground truth -> candidate
        matched = 0
        for gx, gy in gt_arr:
            dists = np.sqrt((cand_arr[:, 0] - gx)**2 + (cand_arr[:, 1] - gy)**2)
            if dists.min() <= match_r:
                matched += 1

        mean_nn = float(nn_dists.mean())
        sorted_nn = np.sort(nn_dists)
        median_nn = float(sorted_nn[len(sorted_nn) // 2])
        max_nn = float(sorted_nn[-1])
        coverage = matched / len(gt) if gt else 0.0
        fp_rate = float(fp_count) / len(cand) if cand else 0.0
        precision = 1.0 - fp_rate
        f1 = 2 * coverage * precision / (coverage + precision) if (coverage + precision) > 0 else 0.0
        point_ratio = len(cand) / len(gt) if gt else 0.0

        self._csv_writer.writerow([
            self._frame_count,
            f'{time.monotonic():.3f}',
            len(gt), len(cand), f'{point_ratio:.3f}',
            f'{mean_nn:.4f}', f'{median_nn:.4f}', f'{max_nn:.4f}',
            f'{coverage:.4f}', f'{fp_rate:.4f}', f'{precision:.4f}', f'{f1:.4f}',
        ])

        if self._frame_count % 25 == 0:
            self._csv_file.flush()
            self.get_logger().info(
                f'[BENCH] frame={self._frame_count} gt={len(gt)} cand={len(cand)} '
                f'mean_nn={mean_nn:.3f}m cov={coverage:.1%} fp={fp_rate:.1%} f1={f1:.3f}')

    def destroy_node(self):
        if self._frame_count > 0:
            self.get_logger().info('='*60)
            self.get_logger().info(f'BENCHMARK SUMMARY: {self._detector}')
            self.get_logger().info(f'  Frames scored: {self._frame_count}')
            self.get_logger().info(f'  Results: {self._csv_path}')
            self.get_logger().info('='*60)
        self._csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LaneBenchmarkNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
