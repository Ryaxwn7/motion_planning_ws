#!/usr/bin/env python3
import argparse
import csv
import math
import os
import statistics
import time
from pathlib import Path

import rospy
from std_msgs.msg import Float64


METRICS = {
    "planning_time": "planning_time",
    "distance_field_update_time": "distance_field_update_time",
    "velocity_map_time": "velocity_map_time",
}


def percentile(values, p):
    if not values:
        return float("nan")
    if len(values) == 1:
        return values[0]
    ordered = sorted(values)
    rank = (len(ordered) - 1) * p
    low = int(math.floor(rank))
    high = int(math.ceil(rank))
    if low == high:
        return ordered[low]
    weight = rank - low
    return ordered[low] * (1.0 - weight) + ordered[high] * weight


class Collector:
    def __init__(self, backend, robot_prefix, robot_count):
        self.backend = backend
        self.robot_prefix = robot_prefix
        self.robot_count = robot_count
        self.rows = []
        self.subs = []

        for robot_id in range(1, robot_count + 1):
            robot_name = f"{robot_prefix}{robot_id}"
            for metric_name, topic_leaf in METRICS.items():
                topic = f"/{robot_name}/move_base/GraphPlanner/{topic_leaf}"
                self.subs.append(
                    rospy.Subscriber(
                        topic,
                        Float64,
                        self._make_cb(robot_name, metric_name, topic),
                        queue_size=100,
                    )
                )

    def _make_cb(self, robot_name, metric_name, topic):
        def _cb(msg):
            self.rows.append(
                {
                    "backend": self.backend,
                    "wall_time": time.time(),
                    "ros_time": rospy.get_time(),
                    "robot": robot_name,
                    "metric": metric_name,
                    "topic": topic,
                    "value_ms": float(msg.data),
                }
            )

        return _cb


def summarize(rows):
    grouped = {}
    for row in rows:
        grouped.setdefault((row["robot"], row["metric"]), []).append(row["value_ms"])

    summary_rows = []
    for (robot, metric), values in sorted(grouped.items()):
        values = sorted(values)
        summary_rows.append(
            {
                "scope": robot,
                "metric": metric,
                "count": len(values),
                "mean_ms": statistics.fmean(values),
                "median_ms": statistics.median(values),
                "p95_ms": percentile(values, 0.95),
                "min_ms": min(values),
                "max_ms": max(values),
            }
        )

    overall_grouped = {}
    for row in rows:
        overall_grouped.setdefault(row["metric"], []).append(row["value_ms"])
    for metric, values in sorted(overall_grouped.items()):
        values = sorted(values)
        summary_rows.append(
            {
                "scope": "ALL",
                "metric": metric,
                "count": len(values),
                "mean_ms": statistics.fmean(values),
                "median_ms": statistics.median(values),
                "p95_ms": percentile(values, 0.95),
                "min_ms": min(values),
                "max_ms": max(values),
            }
        )

    return summary_rows


def write_csv(path, rows, fieldnames):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--backend", required=True)
    parser.add_argument("--duration", type=float, required=True)
    parser.add_argument("--robot-prefix", default="robot")
    parser.add_argument("--robot-count", type=int, default=4)
    parser.add_argument("--raw-csv", required=True)
    parser.add_argument("--summary-csv", required=True)
    args = parser.parse_args()

    rospy.init_node(f"fm2_benchmark_collect_{args.backend}", anonymous=True)
    collector = Collector(args.backend, args.robot_prefix, args.robot_count)

    deadline = time.time() + args.duration
    rate = rospy.Rate(20)
    while not rospy.is_shutdown() and time.time() < deadline:
        rate.sleep()

    raw_path = Path(args.raw_csv)
    summary_path = Path(args.summary_csv)
    write_csv(
        raw_path,
        collector.rows,
        ["backend", "wall_time", "ros_time", "robot", "metric", "topic", "value_ms"],
    )
    write_csv(
        summary_path,
        summarize(collector.rows),
        ["scope", "metric", "count", "mean_ms", "median_ms", "p95_ms", "min_ms", "max_ms"],
    )


if __name__ == "__main__":
    main()
