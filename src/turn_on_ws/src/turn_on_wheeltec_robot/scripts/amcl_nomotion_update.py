#!/usr/bin/env python3

import math
from typing import Optional

import rospy
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty


class AmclNomotionUpdater:
    def __init__(self) -> None:
        self.period = float(rospy.get_param("~period", 1.0))
        self.only_when_stationary = bool(rospy.get_param("~only_when_stationary", True))
        self.linear_threshold = float(rospy.get_param("~linear_threshold", 0.02))
        self.angular_threshold = float(rospy.get_param("~angular_threshold", 0.03))
        self.odom_timeout = float(rospy.get_param("~odom_timeout", 1.0))
        self.service_name = str(rospy.get_param("~service_name", "request_nomotion_update"))
        self.odom_topic = str(rospy.get_param("~odom_topic", "odom"))

        self.last_odom_stamp: Optional[rospy.Time] = None
        self.last_linear_speed = 0.0
        self.last_angular_speed = 0.0

        if self.period <= 0.0:
            raise ValueError("~period must be positive")

        if self.only_when_stationary:
            self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self._odom_cb, queue_size=10)
        else:
            self.odom_sub = None

        self.request_update = rospy.ServiceProxy(self.service_name, Empty)
        rospy.loginfo(
            "amcl_nomotion_update: service=%s period=%.3fs only_when_stationary=%s odom=%s",
            self.service_name,
            self.period,
            str(self.only_when_stationary).lower(),
            self.odom_topic,
        )
        self.timer = rospy.Timer(rospy.Duration(self.period), self._timer_cb)

    def _odom_cb(self, msg: Odometry) -> None:
        vx = float(msg.twist.twist.linear.x)
        vy = float(msg.twist.twist.linear.y)
        vz = float(msg.twist.twist.linear.z)
        wx = float(msg.twist.twist.angular.x)
        wy = float(msg.twist.twist.angular.y)
        wz = float(msg.twist.twist.angular.z)
        self.last_linear_speed = math.sqrt(vx * vx + vy * vy + vz * vz)
        self.last_angular_speed = math.sqrt(wx * wx + wy * wy + wz * wz)
        self.last_odom_stamp = msg.header.stamp if msg.header.stamp != rospy.Time(0) else rospy.Time.now()

    def _is_stationary(self) -> bool:
        if not self.only_when_stationary:
            return True
        if self.last_odom_stamp is None:
            rospy.logwarn_throttle(5.0, "amcl_nomotion_update: waiting for odom on %s", self.odom_topic)
            return False
        if (rospy.Time.now() - self.last_odom_stamp).to_sec() > self.odom_timeout:
            rospy.logwarn_throttle(5.0, "amcl_nomotion_update: odom is stale on %s", self.odom_topic)
            return False
        return self.last_linear_speed <= self.linear_threshold and self.last_angular_speed <= self.angular_threshold

    def _timer_cb(self, _event) -> None:
        if not self._is_stationary():
            return
        try:
            self.request_update()
            rospy.logdebug("amcl_nomotion_update: requested AMCL no-motion update")
        except rospy.ServiceException as exc:
            rospy.logwarn_throttle(
                5.0,
                "amcl_nomotion_update: service call failed for %s: %s",
                self.service_name,
                exc,
            )


def main() -> None:
    rospy.init_node("amcl_nomotion_update")
    AmclNomotionUpdater()
    rospy.spin()


if __name__ == "__main__":
    main()
