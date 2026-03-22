#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

class MoveBaseClient:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base action server")

    def send_goal(self, x, y, yaw):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.z = yaw
        goal.target_pose.pose.orientation.w = 1.0

        rospy.loginfo("Sending goal...")
        self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)

    def done_cb(self, status, result):
        if status == GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal reached!")
        elif status == GoalStatus.ABORTED:
            rospy.loginfo("Goal aborted, trying again...")
            self.cancel_goal()
            self.send_goal(2.0, 2.0, 0.0)
        else:
            rospy.loginfo("Goal failed with status: %s", status)

    def active_cb(self):
        rospy.loginfo("Goal just went active")

    def feedback_cb(self, feedback):
        rospy.loginfo("Feedback received: %s", feedback)

    def cancel_goal(self):
        self.client.cancel_goal()
    def wait_for_result(self):
        self.wait_for_result()

if __name__ == '__main__':
    rospy.init_node('move_base_client')
    client = MoveBaseClient()
    rospy.sleep(5)  # Wait for 5 seconds
    # Send a goal to move_base
    client.send_goal(1.0, 0.0, 0.0)  # Example goal (x, y, yaw)

    # rospy.sleep(5)  # Wait for 5 seconds
    # client.wait_for_result()
    # Cancel the goal
    # client.cancel_goal()
    
    rospy.spin()