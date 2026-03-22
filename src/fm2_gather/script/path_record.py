#! /usr/bin/env python
# -*- coding: utf-8 -*-

import argparse

import rospy
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
    def __init__(self, robot_ids, frame_id, robot_namespace_prefix,
                 odom_topic_suffix, path_topic_suffix, max_path_points):
        self.robot_ids = robot_ids
        self.robot_num = len(robot_ids)
        self.frame_id = frame_id
        self.robot_namespace_prefix = robot_namespace_prefix.strip("/")
        self.odom_topic_suffix = normalize_topic_suffix(odom_topic_suffix, "/odom")
        self.path_topic_suffix = normalize_topic_suffix(path_topic_suffix, "/path")
        self.max_path_points = max_path_points

        # 创建每个机器人的路径发布者和订阅者
        self.path_pubs = {}
        self.odom_subs = {}
        self.paths = {}

        for robot_id in robot_ids:
            # 构建话题名称
            robot_namespace = self.get_robot_namespace(robot_id)
            odom_topic = robot_namespace + self.odom_topic_suffix
            path_topic = robot_namespace + self.path_topic_suffix

            # 创建发布者和订阅者
            self.path_pubs[robot_id] = rospy.Publisher(path_topic, Path, queue_size=10)
            self.odom_subs[robot_id] = rospy.Subscriber(odom_topic, Odometry,
                                                       lambda msg, rid=robot_id: self.odom_callback(msg, rid))

            # 初始化路径
            self.paths[robot_id] = Path()
            self.paths[robot_id].header.frame_id = self.frame_id

            rospy.loginfo("Robot %s: odom=%s, path=%s, frame_id=%s",
                          robot_id, odom_topic, path_topic, self.frame_id)

        rospy.loginfo("Multi-Robot Path Publisher Node Started!")
        rospy.loginfo("Monitoring %d robots: %s", self.robot_num, str(robot_ids))

    def get_robot_namespace(self, robot_id):
        if robot_id == 0:
            return ""
        return "/{0}{1}".format(self.robot_namespace_prefix, robot_id)

    def odom_callback(self, msg, robot_id):
        """ 处理里程计信息，提取位姿并添加到对应机器人的路径 """
        pose_stamped = PoseStamped()

        # 设定时间戳
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = self.frame_id

        # 设定机器人的位姿
        pose_stamped.pose = msg.pose.pose
        pose_stamped.pose.position.z = 0.0

        # 添加位姿到路径
        self.paths[robot_id].poses.append(pose_stamped)

        # 限制路径点数量，避免占用过多内存
        if len(self.paths[robot_id].poses) > self.max_path_points:
            self.paths[robot_id].poses.pop(0)

        # 更新路径时间戳
        self.paths[robot_id].header.stamp = pose_stamped.header.stamp
        self.paths[robot_id].header.frame_id = self.frame_id

        # 发布路径
        self.path_pubs[robot_id].publish(self.paths[robot_id])

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

        frame_id = get_param('~frame_id', 'odom_combined')
        robot_namespace_prefix = get_param('~robot_namespace_prefix', 'robot', 'robot_namespace_prefix')
        odom_topic_suffix = get_param('~odom_topic_suffix', '/odom', 'robot_detect_topic_suffix')
        path_topic_suffix = get_param('~path_topic_suffix', '/path')
        max_path_points = int(get_param('~max_path_points', 10000))

        rospy.loginfo(
            "Path publisher config: frame_id=%s, robot_namespace_prefix=%s, odom_topic_suffix=%s, path_topic_suffix=%s, max_path_points=%d",
            frame_id,
            robot_namespace_prefix,
            odom_topic_suffix,
            path_topic_suffix,
            max_path_points
        )

        path_publisher = PathPublisher(
            robot_ids,
            frame_id,
            robot_namespace_prefix,
            odom_topic_suffix,
            path_topic_suffix,
            max_path_points
        )
        path_publisher.run()
    except rospy.ROSInterruptException:
        pass
