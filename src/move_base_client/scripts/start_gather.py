#!/usr/bin/env python3

import argparse

import rospy
from std_msgs.msg import UInt8


class GatherStateWatcher:
    def __init__(self) -> None:
        self.state = None
        self.sub = rospy.Subscriber("/gather_started", UInt8, self._cb, queue_size=1)

    def _cb(self, msg: UInt8) -> None:
        self.state = int(msg.data)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Trigger fm2_gather to compute and publish a gather center."
    )
    parser.add_argument("--topic", default="/gather_signal", help="Signal topic to publish.")
    parser.add_argument(
        "--signal-data",
        type=int,
        default=2,
        help="Signal value. fm2_gather uses 2 for gather-center compute.",
    )
    parser.add_argument(
        "--wait-connections",
        type=float,
        default=2.0,
        help="Seconds to wait for a subscriber before publishing anyway.",
    )
    parser.add_argument(
        "--repeat",
        type=int,
        default=3,
        help="How many times to publish the signal.",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=5.0,
        help="Publish rate in Hz when repeat > 1.",
    )
    parser.add_argument(
        "--wait-started",
        type=float,
        default=0.0,
        help="Optional seconds to wait for /gather_started == 1 after publishing.",
    )
    return parser.parse_args(rospy.myargv()[1:])


def main() -> int:
    rospy.init_node("start_gather", anonymous=True)
    args = parse_args()

    watcher = GatherStateWatcher()
    pub = rospy.Publisher(args.topic, UInt8, queue_size=1)

    wait_deadline = rospy.Time.now() + rospy.Duration(max(0.0, args.wait_connections))
    while not rospy.is_shutdown():
        if pub.get_num_connections() > 0:
            break
        if rospy.Time.now() >= wait_deadline:
            rospy.logwarn(
                "start_gather: no subscribers on %s after %.2fs, publish anyway.",
                args.topic,
                max(0.0, args.wait_connections),
            )
            break
        rospy.sleep(0.05)

    msg = UInt8()
    msg.data = int(args.signal_data)
    repeat = max(1, int(args.repeat))
    rate_hz = max(0.1, float(args.rate))
    rate = rospy.Rate(rate_hz)
    for idx in range(repeat):
        if rospy.is_shutdown():
            return 1
        pub.publish(msg)
        rospy.loginfo(
            "start_gather: published %s=%d (%d/%d)",
            args.topic,
            msg.data,
            idx + 1,
            repeat,
        )
        if idx + 1 < repeat:
            rate.sleep()

    wait_started = max(0.0, float(args.wait_started))
    if wait_started <= 0.0:
        return 0

    started_deadline = rospy.Time.now() + rospy.Duration(wait_started)
    while not rospy.is_shutdown() and rospy.Time.now() < started_deadline:
        if watcher.state == 1:
            rospy.loginfo("start_gather: /gather_started became 1.")
            return 0
        rospy.sleep(0.05)

    rospy.logwarn(
        "start_gather: timed out waiting %.2fs for /gather_started == 1 (last state=%s).",
        wait_started,
        "none" if watcher.state is None else str(watcher.state),
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
