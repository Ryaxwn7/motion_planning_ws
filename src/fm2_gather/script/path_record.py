#! /usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import time
from collections import deque

import rospy
import tf2_geometry_msgs  # Registers geometry_msgs conversions with tf2_ros.
import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry

"""
机器人路径发布者：根据 odom 话题，实时更新机器人的轨迹，发布到 path 话题。

"""


def get_param(private_name, default_value, global_name=None):
    if rospy.has_param(private_name):
        return rospy.get_param(private_name)
    if global_name and rospy.has_param(global_name):
        return rospy.get_param(global_name)
    return default_value


def normalize_topic_suffix(topic_suffix, default_suffix):
    suffix = topic_suffix if topic_suffix else default_suffix
    if not suffix.startswith("/"):
        suffix = "/" + suffix
    return suffix


class PathPublisher:
    def __init__(self, robot_ids, frame_id, target_frame, pose_source,
                 base_frame_suffix, transform_timeout, robot_namespace_prefix,
                 odom_topic_suffix, path_topic_suffix, max_path_points):
        self.robot_ids = robot_ids
        self.robot_num = len(robot_ids)
        self.frame_id = frame_id
        self.target_frame = target_frame.strip().lstrip("/")
        self.pose_source = pose_source.strip().lower()
        self.base_frame_suffix = base_frame_suffix.strip().strip("/")
        self.transform_timeout = transform_timeout
        self.robot_namespace_prefix = robot_namespace_prefix.strip("/")
        self.odom_topic_suffix = normalize_topic_suffix(odom_topic_suffix, "/odom")
        self.path_topic_suffix = normalize_topic_suffix(path_topic_suffix, "/path")
        self.max_path_points = max_path_points

        if not self.target_frame:
            raise ValueError("target_frame must not be empty")
        if self.pose_source not in ("tf", "odom"):
            raise ValueError("pose_source must be 'tf' or 'odom'")
        if self.pose_source == "tf" and not self.base_frame_suffix:
            raise ValueError("base_frame_suffix must not be empty in tf pose mode")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # 创建每个机器人的路径发布者和订阅者
        self.path_pubs = {}
        self.odom_subs = {}
        self.paths = {}
        self.pending_odom = {}

        for robot_id in robot_ids:
            # 构建话题名称
            robot_namespace = self.get_robot_namespace(robot_id)
            odom_topic = robot_namespace + self.odom_topic_suffix
            path_topic = robot_namespace + self.path_topic_suffix

            # Initialize state before subscribing because callbacks may run immediately.
            self.paths[robot_id] = Path()
            self.paths[robot_id].header.frame_id = self.target_frame
            self.pending_odom[robot_id] = deque()

            # 创建发布者和订阅者
            self.path_pubs[robot_id] = rospy.Publisher(path_topic, Path, queue_size=10)
            self.odom_subs[robot_id] = rospy.Subscriber(odom_topic, Odometry,
                                                       lambda msg, rid=robot_id: self.odom_callback(msg, rid))

            rospy.loginfo(
                "Robot %s: odom=%s, path=%s, pose_source=%s, tracking_frame=%s, target_frame=%s",
                robot_id,
                odom_topic,
                path_topic,
                self.pose_source,
                self.get_tracking_frame(robot_id),
                self.target_frame
            )

        rospy.loginfo("Multi-Robot Path Publisher Node Started!")
        rospy.loginfo("Monitoring %d robots: %s", self.robot_num, str(robot_ids))

    def get_robot_namespace(self, robot_id):
        if robot_id == 0:
            return ""
        return "/{0}{1}".format(self.robot_namespace_prefix, robot_id)

    def resolve_frame_id(self, robot_id, msg_frame_id):
        """Resolve a valid TF frame for each robot's Path message."""
        configured_frame = self.frame_id.strip().lstrip("/")
        incoming_frame = msg_frame_id.strip().lstrip("/")

        if not configured_frame or configured_frame.lower() == "auto":
            if incoming_frame:
                return incoming_frame
            configured_frame = "odom_combined"

        # Global and already-qualified frames must not receive a robot prefix.
        if configured_frame in ("map", "world", "earth") or "/" in configured_frame:
            return configured_frame

        robot_namespace = self.get_robot_namespace(robot_id).strip("/")
        if robot_namespace:
            return robot_namespace + "/" + configured_frame
        return configured_frame

    def get_tracking_frame(self, robot_id):
        if "/" in self.base_frame_suffix:
            return self.base_frame_suffix
        robot_namespace = self.get_robot_namespace(robot_id).strip("/")
        if robot_namespace:
            return robot_namespace + "/" + self.base_frame_suffix
        return self.base_frame_suffix

    @staticmethod
    def get_message_stamp(msg):
        if msg.header.stamp != rospy.Time():
            return msg.header.stamp
        return rospy.Time.now()

    def make_source_pose(self, msg, robot_id):
        source_pose = PoseStamped()
        source_frame = self.resolve_frame_id(robot_id, msg.header.frame_id)

        source_pose.header.stamp = self.get_message_stamp(msg)
        source_pose.header.frame_id = source_frame
        source_pose.pose = msg.pose.pose
        return source_pose

    def make_tf_pose(self, robot_id, stamp):
        tracking_frame = self.get_tracking_frame(robot_id)
        transform = self.tf_buffer.lookup_transform(
            self.target_frame, tracking_frame, stamp, rospy.Duration(0.0))
        target_pose = PoseStamped()
        target_pose.header.stamp = stamp
        target_pose.header.frame_id = self.target_frame
        target_pose.pose.position.x = transform.transform.translation.x
        target_pose.pose.position.y = transform.transform.translation.y
        target_pose.pose.position.z = transform.transform.translation.z
        target_pose.pose.orientation = transform.transform.rotation
        return target_pose

    def append_target_pose(self, robot_id, target_pose):
        self.paths[robot_id].poses.append(target_pose)

        if len(self.paths[robot_id].poses) > self.max_path_points:
            self.paths[robot_id].poses.pop(0)

        self.paths[robot_id].header.stamp = target_pose.header.stamp
        self.paths[robot_id].header.frame_id = self.target_frame

    def process_pending_odom(self, robot_id):
        """Transform queued odometry once TF data for its timestamp is available."""
        pending = self.pending_odom[robot_id]
        path_updated = False

        while pending:
            msg, queued_at = pending[0]
            stamp = self.get_message_stamp(msg)
            if self.pose_source == "tf":
                source_pose = None
                source_frame = self.get_tracking_frame(robot_id)
            else:
                source_pose = self.make_source_pose(msg, robot_id)
                source_frame = source_pose.header.frame_id

            if source_frame != self.target_frame and not self.tf_buffer.can_transform(
                    self.target_frame, source_frame, stamp, rospy.Duration(0.0)):
                if time.monotonic() - queued_at < self.transform_timeout:
                    break
                pending.popleft()
                rospy.logwarn_throttle(
                    2.0,
                    "Robot %s: dropped odom pose because TF %s -> %s was unavailable at %.6f" % (
                        robot_id,
                        source_frame,
                        self.target_frame,
                        stamp.to_sec()
                    )
                )
                continue

            pending.popleft()
            try:
                if self.pose_source == "tf":
                    target_pose = self.make_tf_pose(robot_id, stamp)
                elif source_frame == self.target_frame:
                    target_pose = source_pose
                else:
                    target_pose = self.tf_buffer.transform(
                        source_pose, self.target_frame, rospy.Duration(0.0))
            except tf2_ros.TransformException as error:
                rospy.logwarn_throttle(
                    2.0,
                    "Robot %s: failed to resolve pose from %s to %s at %.6f: %s" % (
                        robot_id,
                        source_frame,
                        self.target_frame,
                        stamp.to_sec(),
                        error
                    )
                )
                continue

            # The map and trajectory visualization are planar.
            target_pose.pose.position.z = 0.0
            self.append_target_pose(robot_id, target_pose)
            path_updated = True

        if path_updated:
            self.path_pubs[robot_id].publish(self.paths[robot_id])

    def odom_callback(self, msg, robot_id):
        """Queue odometry and convert ready poses into the target frame."""
        self.pending_odom[robot_id].append((msg, time.monotonic()))
        self.process_pending_odom(robot_id)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        # 解析命令行参数
        ap = argparse.ArgumentParser()
        ap.add_argument("-r", "--robot_ids", nargs='+', type=int,
                        help="List of robot IDs to monitor (e.g., 1 2 3)")
        args = ap.parse_args(rospy.myargv()[1:])

        rospy.init_node('multi_robot_path_publisher', anonymous=True)

        # 如果没有提供机器人ID，尝试从参数服务器获取
        if args.robot_ids is None:
            robot_ids = get_param('~robot_ids', [1, 2, 3], 'robot_ids')
            robot_ids = [int(robot_id) for robot_id in robot_ids]
            rospy.loginfo("Using default robot configuration: %d robots with IDs %s", 
                          len(robot_ids), str(robot_ids))
        else:
            robot_ids = args.robot_ids
            rospy.loginfo("Using provided robot IDs: %s", str(robot_ids))

        frame_id = get_param('~frame_id', 'auto')
        target_frame = get_param('~target_frame', 'map')
        pose_source = get_param('~pose_source', 'tf')
        base_frame_suffix = get_param('~base_frame_suffix', 'base_footprint')
        transform_timeout = float(get_param('~transform_timeout', 0.5))
        robot_namespace_prefix = get_param('~robot_namespace_prefix', 'robot', 'robot_namespace_prefix')
        odom_topic_suffix = get_param('~odom_topic_suffix', '/odom', 'robot_detect_topic_suffix')
        path_topic_suffix = get_param('~path_topic_suffix', '/path')
        max_path_points = int(get_param('~max_path_points', 10000))

        rospy.loginfo(
            "Path publisher config: pose_source=%s, source_frame=%s, base_frame_suffix=%s, target_frame=%s, transform_timeout=%.3f, robot_namespace_prefix=%s, odom_topic_suffix=%s, path_topic_suffix=%s, max_path_points=%d",
            pose_source,
            frame_id,
            base_frame_suffix,
            target_frame,
            transform_timeout,
            robot_namespace_prefix,
            odom_topic_suffix,
            path_topic_suffix,
            max_path_points
        )

        path_publisher = PathPublisher(
            robot_ids,
            frame_id,
            target_frame,
            pose_source,
            base_frame_suffix,
            transform_timeout,
            robot_namespace_prefix,
            odom_topic_suffix,
            path_topic_suffix,
            max_path_points
        )
        path_publisher.run()
    except rospy.ROSInterruptException:
        pass
