#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from collections import deque

import numpy as np
import rclpy
from bisa.msg import MPCPerformance
from rclpy.node import Node


class PerformanceAnalyzer(Node):
    def __init__(self):
        super().__init__('performance_analyzer')

        self.total_times = deque(maxlen=1000)
        self.model_times = deque(maxlen=1000)
        self.solver_times = deque(maxlen=1000)

        self.create_subscription(
            MPCPerformance,
            '/mpc_performance',
            self.perf_callback,
            10
        )

        self.timer = self.create_timer(5.0, self.print_stats)

    def perf_callback(self, msg):
        self.total_times.append(msg.total_time_us)
        self.model_times.append(msg.model_time_us)
        self.solver_times.append(msg.solver_time_us)

    def print_stats(self):
        if not self.total_times:
            return

        total = np.array(self.total_times)
        model = np.array(self.model_times)
        solver = np.array(self.solver_times)

        print("\n" + "=" * 60)
        print("MPC PERFORMANCE STATISTICS")
        print("=" * 60)
        print(f"{'Metric':<20} {'Mean':>10} {'P50':>10} {'P95':>10} {'P99':>10} {'Max':>10}")
        print("-" * 60)
        print(f"{'Total (us)':<20} {total.mean():>10.1f} {np.percentile(total, 50):>10.1f} "
              f"{np.percentile(total, 95):>10.1f} {np.percentile(total, 99):>10.1f} {total.max():>10.1f}")
        print(f"{'Model (us)':<20} {model.mean():>10.1f} {np.percentile(model, 50):>10.1f} "
              f"{np.percentile(model, 95):>10.1f} {np.percentile(model, 99):>10.1f} {model.max():>10.1f}")
        print(f"{'Solver (us)':<20} {solver.mean():>10.1f} {np.percentile(solver, 50):>10.1f} "
              f"{np.percentile(solver, 95):>10.1f} {np.percentile(solver, 99):>10.1f} {solver.max():>10.1f}")
        print("=" * 60)

        # Check 1kHz compliance (1000us = 1ms)
        violations = (total > 1000).sum()
        violation_rate = violations / len(total) * 100
        print(f"1kHz Violations: {violations}/{len(total)} ({violation_rate:.2f}%)")

        if violation_rate > 5:
            print("WARNING: >5% violations. Consider:")
            print("  - Reduce horizon")
            print("  - Optimize solver settings")
            print("  - Use sparse matrices")
        elif violation_rate > 0:
            print("OK: Acceptable performance (<5% violations)")
        else:
            print("OK: No violations")
        print("=" * 60 + "\n")


def main():
    rclpy.init()
    analyzer = PerformanceAnalyzer()
    rclpy.spin(analyzer)
    analyzer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
