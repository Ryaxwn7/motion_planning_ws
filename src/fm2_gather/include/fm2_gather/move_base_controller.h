#ifndef MOVE_BASE_CONTROLLER_H
#define MOVE_BASE_CONTROLLER_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>
#include <std_msgs/UInt8.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class MoveBaseController {
public:
    // 构造函数（带命名空间参数）
    MoveBaseController(const std::string& robot_namespace);
    
    // 核心功能接口
    bool sendGoal(const move_base_msgs::MoveBaseGoal& goal);
    void cancelGoal();
    void emergencyStop();
    actionlib::SimpleClientGoalState getState() const;
    bool isStopped() const;
    void resendGoal();
    bool checkConnection() const;
    bool makePlan(geometry_msgs::PoseStamped robot_pose, const geometry_msgs::PoseStamped& goal,  nav_msgs::Path& plan);
private:
    // Action客户端回调
    void doneCallback(const actionlib::SimpleClientGoalState& state,
                      const move_base_msgs::MoveBaseResultConstPtr& result);
    void activeCallback();
    void feedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);
    void reconnectTimerCallback(const ros::TimerEvent& event);

    // 成员变量
    MoveBaseClient ac_;
    ros::Publisher vel_pub_;
    ros::Publisher state_pub_;
    std::string namespace_;
    std_msgs::UInt8 msg_;
    move_base_msgs::MoveBaseGoal current_goal_;
    bool new_goal_flag_;
    ros::Timer reconnect_timer_;
    std::string mb_service_name_;
    ros::ServiceClient client_;
};

#endif // MOVE_BASE_CONTROLLER_H