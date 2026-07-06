#!/usr/bin/env python3

import argparse
import math
from collections import OrderedDict

import rosbag


ODOM_TYPE = "nav_msgs/Odometry"


def point_distance(a, b, include_z):
    dx = a.x - b.x
    dy = a.y - b.y
    if not include_z:
        return math.sqrt(dx * dx + dy * dy)

    dz = a.z - b.z
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def is_finite_position(position):
    return (
        math.isfinite(position.x)
        and math.isfinite(position.y)
        and math.isfinite(position.z)
    )


def odom_topics(bag):
    topic_info = bag.get_type_and_topic_info()[1]
    return sorted(
        topic for topic, info in topic_info.items() if info.msg_type == ODOM_TYPE
    )


def calculate_topic_length(bag, topic, include_z, min_distance, max_step):
    last_position = None
    first_stamp = None
    last_stamp = None
    poses = 0
    used_segments = 0
    skipped_small = 0
    skipped_jump = 0
    skipped_invalid = 0
    length = 0.0

    for _, msg, stamp in bag.read_messages(topics=[topic]):
        position = msg.pose.pose.position
        if not is_finite_position(position):
            skipped_invalid += 1
            continue

        poses += 1
        msg_stamp = msg.header.stamp if not msg.header.stamp.is_zero() else stamp
        if first_stamp is None:
            first_stamp = msg_stamp

        if last_position is not None:
            step = point_distance(last_position, position, include_z)
            if min_distance > 0.0 and step < min_distance:
                skipped_small += 1
            elif max_step > 0.0 and step > max_step:
                skipped_jump += 1
            else:
                length += step
                used_segments += 1

        last_position = position
        last_stamp = msg_stamp

    duration = 0.0
    if first_stamp is not None and last_stamp is not None:
        duration = (last_stamp - first_stamp).to_sec()

    return OrderedDict(
        [
            ("topic", topic),
            ("poses", poses),
            ("segments", used_segments),
            ("length", length),
            ("duration", duration),
            ("skipped_small", skipped_small),
            ("skipped_jump", skipped_jump),
            ("skipped_invalid", skipped_invalid),
        ]
    )


def format_result(result):
    return (
        "{topic}: poses={poses} segments={segments} length={length:.6f}m "
        "duration={duration:.3f}s skipped_small={skipped_small} "
        "skipped_jump={skipped_jump} skipped_invalid={skipped_invalid}"
    ).format(**result)


def main():
    parser = argparse.ArgumentParser(
        description="Calculate robot trajectory length from nav_msgs/Odometry messages in a rosbag."
    )
    parser.add_argument("bag", help="Input rosbag file.")
    parser.add_argument(
        "-t",
        "--topic",
        action="append",
        dest="topics",
        help="Odometry topic to process. Can be used multiple times. Defaults to all nav_msgs/Odometry topics.",
    )
    parser.add_argument(
        "--include-z",
        action="store_true",
        help="Use 3D distance. By default only x/y planar distance is accumulated.",
    )
    parser.add_argument(
        "--min-distance",
        type=float,
        default=0.0,
        help="Ignore steps shorter than this distance in meters, useful for odom jitter.",
    )
    parser.add_argument(
        "--max-step",
        type=float,
        default=0.0,
        help="Ignore steps longer than this distance in meters, useful for localization jumps. 0 disables filtering.",
    )
    args = parser.parse_args()

    if args.min_distance < 0.0:
        raise SystemExit("--min-distance must be >= 0")
    if args.max_step < 0.0:
        raise SystemExit("--max-step must be >= 0")

    with rosbag.Bag(args.bag, "r") as bag:
        available_odom_topics = odom_topics(bag)
        topics = args.topics or available_odom_topics
        if not topics:
            raise SystemExit("no nav_msgs/Odometry topics found in bag")

        missing = sorted(set(topics) - set(available_odom_topics))
        if missing:
            raise SystemExit(
                "requested topic(s) are not nav_msgs/Odometry in this bag: "
                + ", ".join(missing)
            )

        total_length = 0.0
        for topic in topics:
            result = calculate_topic_length(
                bag,
                topic,
                include_z=args.include_z,
                min_distance=args.min_distance,
                max_step=args.max_step,
            )
            total_length += result["length"]
            print(format_result(result))

        if len(topics) > 1:
            print("total_length={:.6f}m".format(total_length))


if __name__ == "__main__":
    main()
