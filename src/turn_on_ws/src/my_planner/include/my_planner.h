#ifndef MY_PLANNER_H
#define MY_PLANNER_H

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <dynamic_reconfigure/server.h>
#include <my_planner/MyPlannerConfig.h>

namespace my_planner
{
    class MyPlanner : public nav_core::BaseLocalPlanner
    {
        public:
            MyPlanner();
            ~MyPlanner();

            void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);
            bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);
            bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
            void reconfigureCB(MyPlannerConfig& config, uint32_t level);
            bool isGoalReached();
            void applyAccelerationLimits(geometry_msgs::Twist& cmd_vel);

            dynamic_reconfigure::Server<my_planner::MyPlannerConfig>* dyn_server_; //定义动态参数服务器
            dynamic_reconfigure::Server<my_planner::MyPlannerConfig>::CallbackType dyn_cb_; //定义动态参数回调函数
            ros::Publisher target_pose_pub_;

            bool setup_ = false;
            MyPlannerConfig default_config_;
            bool debug_print_ = true;

            int pre_n;
            double Ki;
            double Kp;
            double Kd;
            double Kp_x;
            double Ki_x;
            double Kd_x;
            double Kp_y;
            double Ki_y;
            double Kd_y;

            double trans_x_factor;
            double trans_y_factor;
            double trans_factor;
            double adjust_r_factor;
            double m_target_dist;
            double m_max_vel_trans;
            double m_max_vel_rot;
            double m_acc_scale_trans;
            double m_acc_scale_rot;
            double m_goal_dist_tolerance;
            double m_goal_yaw_tolerance;
            std::string m_scan_topic;
            std::string m_base_frame_id;
            std::string m_odom_frame_id;

            geometry_msgs::Twist last_cmd_;
            ros::Time last_cmd_time_;
            bool has_last_cmd_ = false;
            





    };

}// namespace my_planner

#endif // MY_PLANNER_H
