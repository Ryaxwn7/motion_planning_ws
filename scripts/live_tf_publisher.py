#!/usr/bin/env python3

import math

import rospy
import tf.transformations
import tf2_ros
from geometry_msgs.msg import TransformStamped


def _param(name, default):
    return rospy.get_param("~" + name, default)


def main() -> int:
    rospy.init_node("live_tf_publisher")

    parent_frame = str(_param("parent_frame", "map")).strip() or "map"
    child_frame = str(_param("child_frame", "world")).strip() or "world"
    rate_hz = float(_param("rate", 20.0))
    if rate_hz <= 0.0:
        rate_hz = 20.0

    broadcaster = tf2_ros.TransformBroadcaster()
    rate = rospy.Rate(rate_hz)

    rospy.loginfo(
        "live_tf_publisher: publishing %s -> %s, edit params under /%s",
        parent_frame,
        child_frame,
        rospy.get_name().strip("/"),
    )

    while not rospy.is_shutdown():
        parent_frame = str(_param("parent_frame", parent_frame)).strip() or parent_frame
        child_frame = str(_param("child_frame", child_frame)).strip() or child_frame
        x = float(_param("x", 0.0))
        y = float(_param("y", 0.0))
        z = float(_param("z", 0.0))
        roll = float(_param("roll", 0.0))
        pitch = float(_param("pitch", 0.0))
        yaw = float(_param("yaw", 0.0))

        quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

        msg = TransformStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = parent_frame
        msg.child_frame_id = child_frame
        msg.transform.translation.x = x
        msg.transform.translation.y = y
        msg.transform.translation.z = z
        msg.transform.rotation.x = quat[0]
        msg.transform.rotation.y = quat[1]
        msg.transform.rotation.z = quat[2]
        msg.transform.rotation.w = quat[3]
        broadcaster.sendTransform(msg)
        rate.sleep()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
