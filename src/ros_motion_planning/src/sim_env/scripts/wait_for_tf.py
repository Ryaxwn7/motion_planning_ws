#!/usr/bin/env python3

import os
import sys
import time

import rospy
import tf2_ros


def _wait_for_tf(target_frame: str, source_frame: str, timeout_s: float, check_period_s: float) -> bool:
    buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
    tf2_ros.TransformListener(buffer)

    start_time = time.monotonic()
    warned_timeout = False
    rate = rospy.Rate(max(1.0, 1.0 / check_period_s))

    rospy.loginfo("Waiting for TF %s -> %s (timeout=%.1fs)", target_frame, source_frame, timeout_s)
    while not rospy.is_shutdown():
        if buffer.can_transform(target_frame, source_frame, rospy.Time(0), rospy.Duration(0.1)):
            rospy.loginfo("TF ready: %s -> %s", target_frame, source_frame)
            return True

        if timeout_s > 0.0 and not warned_timeout and (time.monotonic() - start_time) >= timeout_s:
            rospy.logwarn("Still waiting for TF %s -> %s after %.1fs", target_frame, source_frame, timeout_s)
            warned_timeout = True

        rate.sleep()

    return False


def main() -> None:
    rospy.init_node("wait_for_tf_prefix", anonymous=True, disable_signals=True)

    if len(sys.argv) < 4:
        target_frame = rospy.get_param("~target_frame", "map")
        source_frame = rospy.get_param("~source_frame", "odom")
        timeout_s = float(rospy.get_param("~timeout", 30.0))
        check_period_s = float(rospy.get_param("~check_period", 0.1))
        _wait_for_tf(target_frame, source_frame, timeout_s, check_period_s)
        return

    target_frame = sys.argv[1]
    source_frame = sys.argv[2]
    timeout_s = float(sys.argv[3])
    actual_command = sys.argv[4:]

    if not actual_command:
        rospy.logerr("No command supplied after wait_for_tf arguments")
        return

    ok = _wait_for_tf(target_frame, source_frame, timeout_s, 0.1)
    if not ok:
        sys.exit(1)

    rospy.loginfo("Starting command: %s", " ".join(actual_command))
    os.execv(actual_command[0], actual_command)


if __name__ == "__main__":
    main()
