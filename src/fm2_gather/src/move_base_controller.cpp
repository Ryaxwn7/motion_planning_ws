#include "fm2_gather/move_base_controller.h"
#include <ros/console.h>

MoveBaseController::MoveBaseController(const std::string& robot_namespace)
    : ac_(robot_namespace + "/move_base", true),
      namespace_(robot_namespace),
      new_goal_flag_(false)
      
{
    ros::NodeHandle nh;
    vel_pub_ = nh.advertise<geometry_msgs::Twist>(namespace_ + "/cmd_vel", 1);
    state_pub_ = nh.advertise<std_msgs::UInt8>(namespace_ + "/mv_state", 1);
    mb_service_name_= (robot_namespace + "/move_base/make_plan");
    while(!ac_.waitForServer(ros::Duration(3.0))){
        ROS_ERROR_STREAM("[" << namespace_ << "] Failed to connect to move_base server!Waiting for 3s...");
    }
    reconnect_timer_ = nh.createTimer(ros::Duration(1.0), &MoveBaseController::reconnectTimerCallback, this);
    while(!ros::service::waitForService(mb_service_name_, ros::Duration(3.0)))
    {
        ROS_INFO_STREAM("[" << namespace_ << "] Waiting for make_plan service...");
    }
    client_ = nh.serviceClient<nav_msgs::GetPlan>(mb_service_name_,true);
    if(!client_)
    {
        ROS_FATAL_STREAM("[" << namespace_ << "] Failed to create make_plan service client!");
        exit(1);
    }
    
}

bool MoveBaseController::sendGoal(const move_base_msgs::MoveBaseGoal& goal) {
    // if(ac_.getState().isDone()) {
    //     ac_.cancelAllGoals();
    // }

    ac_.sendGoal(goal,
                boost::bind(&MoveBaseController::doneCallback, this, _1, _2),
                boost::bind(&MoveBaseController::activeCallback, this),
                boost::bind(&MoveBaseController::feedbackCallback, this, _1));
    current_goal_ = goal;
    new_goal_flag_ = true;
    return true;
}

 // todo: 实现makePlan函数
bool MoveBaseController::makePlan(geometry_msgs::PoseStamped robotpose, const geometry_msgs::PoseStamped& goal, nav_msgs::Path& plan) 
{
    // 调用服务
    nav_msgs::GetPlan srv;
    srv.request.start.header.frame_id = robotpose.header.frame_id;
    srv.request.start.pose = robotpose.pose;
    srv.request.goal.header.frame_id = goal.header.frame_id;
    srv.request.goal.pose = goal.pose;
    if(client_.call(srv))
    {
        if(!srv.response.plan.poses.empty())
        {
            plan = srv.response.plan;
            return true;
        }
        else 
        {
            ROS_WARN_STREAM("[" << namespace_ << "] Failed to get plan!");
            return false;
        }
    }
    else
    {
        ROS_ERROR_STREAM("[" << namespace_ << "] Failed to call make_plan service!");
        return false;
    }
}

void MoveBaseController::resendGoal() {
    ac_.cancelAllGoals();
    if (!ac_.waitForResult(ros::Duration(1.0))) {
        ROS_WARN_STREAM("[" << namespace_ << "] Cancel timeout, force resend");
    }
    ac_.sendGoal(current_goal_,
                boost::bind(&MoveBaseController::doneCallback, this, _1, _2),
                boost::bind(&MoveBaseController::activeCallback, this),
                boost::bind(&MoveBaseController::feedbackCallback, this, _1));
    new_goal_flag_ = false;
}

void MoveBaseController::cancelGoal() {
    ac_.cancelAllGoals();
}

void MoveBaseController::emergencyStop() {
    if(!ac_.getState().isDone()) {
        geometry_msgs::Twist cmd;
        cmd.linear.x = 0;
        cmd.angular.z = 0;
        vel_pub_.publish(cmd);
        ac_.cancelAllGoals();
        if (!ac_.waitForResult(ros::Duration(1.0))) {
            ROS_WARN_STREAM("[" << namespace_ << "] Cancel timeout, force resend");
        }
        ROS_WARN_STREAM("[" << namespace_ << "] Emergency stop!");
    }
}

actionlib::SimpleClientGoalState MoveBaseController::getState() const {
    return ac_.getState();
}

bool MoveBaseController::isStopped() const {
    return ac_.getState().isDone();
}

// 回调函数实现
void MoveBaseController::doneCallback(const actionlib::SimpleClientGoalState& state,
                                    const move_base_msgs::MoveBaseResultConstPtr& result) {
    // 状态映射到UInt8
    if(state == actionlib::SimpleClientGoalState::SUCCEEDED) {
        msg_.data = 4;
        ROS_INFO_STREAM("[" << namespace_ << "] Goal reached!");
    } else if(state == actionlib::SimpleClientGoalState::PREEMPTED) {
        ROS_WARN_STREAM("[" << namespace_ << "] Goal preempted!");
        msg_.data = 3;
    } else if(state == actionlib::SimpleClientGoalState::ABORTED) {
        msg_.data = 5;
    }
    state_pub_.publish(msg_);
}

void MoveBaseController::activeCallback() {
    msg_.data = 1;
    state_pub_.publish(msg_);
}

void MoveBaseController::feedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) {
    // 可选：实现反馈处理
    // // 如果需要可以在此处发布运行中的状态（如果msg_.data=1表示运行中）
    // msg_.data = 1;
    // state_pub_.publish(msg_);
}

bool MoveBaseController::checkConnection() const {
    return ac_.isServerConnected();
}

void MoveBaseController::reconnectTimerCallback(const ros::TimerEvent& event) {
    if(!ac_.isServerConnected()) {
        ROS_WARN_STREAM("[" << namespace_ << "] Reconnect to move_base server...");
        ac_.waitForServer(ros::Duration(5.0));
    }
}