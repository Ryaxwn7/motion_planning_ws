#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import argparse

"""
机器人路径发布者：根据/odom话题，实时更新机器人的轨迹，发布到/path话题

"""

class PathPublisher:
    def __init__(self, robot_ids):
        # 初始化ROS节点
        rospy.init_node('multi_robot_path_publisher', anonymous=True)
        
        # 从参数服务器获取机器人数量
        self.robot_num = len(robot_ids)
        self.robot_ids = robot_ids
        
        # 创建每个机器人的路径发布者和订阅者
        self.path_pubs = {}
        self.odom_subs = {}
        self.paths = {}
        
        for robot_id in robot_ids:
            # 构建话题名称
            prefix = "/robot" + str(robot_id) if robot_id != 0 else ""
            odom_topic = prefix + "/odom"
            path_topic = prefix + "/path"
            
            # 创建发布者和订阅者
            self.path_pubs[robot_id] = rospy.Publisher(path_topic, Path, queue_size=10)
            self.odom_subs[robot_id] = rospy.Subscriber(odom_topic, Odometry, 
                                                       lambda msg, rid=robot_id: self.odom_callback(msg, rid))
            
            # 初始化路径
            self.paths[robot_id] = Path()
            self.paths[robot_id].header.frame_id = "map"  # 使用map作为参考坐标系
            
        rospy.loginfo("Multi-Robot Path Publisher Node Started!")
        rospy.loginfo("Monitoring %d robots: %s", self.robot_num, str(robot_ids))

    def odom_callback(self, msg, robot_id):
        """ 处理里程计信息，提取位姿并添加到对应机器人的路径 """
        pose_stamped = PoseStamped()

        # 设定时间戳
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = "odom_combined"

        # 设定机器人的位姿
        pose_stamped.pose = msg.pose.pose
        pose_stamped.pose.position.z = 0.0

        # 添加位姿到路径
        self.paths[robot_id].poses.append(pose_stamped)

        # 限制路径点数量，避免占用过多内存
        if len(self.paths[robot_id].poses) > 10000:
            self.paths[robot_id].poses.pop(0)

        # 更新路径时间戳
        self.paths[robot_id].header.stamp = rospy.Time.now()

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
        args = ap.parse_args()
        
        # 如果没有提供机器人ID，尝试从参数服务器获取
        if args.robot_ids is None:
            rospy.init_node('multi_robot_path_publisher', anonymous=True)
            robot_num = rospy.get_param('num_robots', 3)
            robot_ids = rospy.get_param('robot_ids', [1, 2, 3])
            rospy.loginfo("Using default robot configuration: %d robots with IDs %s", 
                         robot_num, str(robot_ids))
        else:
            robot_ids = args.robot_ids
            rospy.loginfo("Using provided robot IDs: %s", str(robot_ids))
        
        path_publisher = PathPublisher(robot_ids)
        path_publisher.run()
    except rospy.ROSInterruptException:
        pass