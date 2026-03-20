#include "my_planner.h"
#include <pluginlib/class_list_macros.h>


PLUGINLIB_EXPORT_CLASS(my_planner::MyPlanner, nav_core::BaseLocalPlanner)


//PID控制变量
double angular_error = 0.0;
double error_sum = 0.0;
double last_error = 0.0;
double error_diff = 0.0;
double output = 0.0;

namespace my_planner
{
    MyPlanner::MyPlanner()
    {
        setlocale(LC_ALL, "");

    }
    MyPlanner::~MyPlanner()
    {

    }

    tf2_ros::Buffer* tf_buffer_; //定义 tf 缓存器
    tf2_ros::TransformListener* tf_listener_; //定义 tf 监听器

    costmap_2d::Costmap2DROS* costmap_ros_; //定义 costmap ros 指针

    void MyPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
    {
        // Initialize the planner here
        tf_buffer_ = new tf2_ros::Buffer();
        tf_listener_ = new tf2_ros::TransformListener(*tf_buffer_);
        costmap_ros_ = costmap_ros;
        

        ros::NodeHandle nh_planner("~/" + name);
        // Initialize the dynamic reconfigure server
        dyn_server_ = new dynamic_reconfigure::Server<my_planner::MyPlannerConfig>(nh_planner);
        dyn_cb_ = boost::bind(&MyPlanner::reconfigureCB, this, _1, _2);
        dyn_server_->setCallback(dyn_cb_);
        // Load parameters
        nh_planner.param("pre_n", pre_n, 10);
        nh_planner.param("Kp", Kp, 1.4);
        nh_planner.param("Ki", Ki, 0.0);
        nh_planner.param("Kd", Kd, 1.1);
        nh_planner.param("trans_x_factor", trans_x_factor, 1.5);
        nh_planner.param("trans_y_factor", trans_y_factor, 1.5);
        nh_planner.param("adjust_r_factor", adjust_r_factor, 1.0);
        nh_planner.param("max_vel_trans", m_max_vel_trans, 1.0);
        nh_planner.param("max_vel_rot", m_max_vel_rot, 0.9);
        nh_planner.param("acc_scale_trans", m_acc_scale_trans, 1.5);
        nh_planner.param("acc_scale_rot", m_acc_scale_rot, 0.5);
        nh_planner.param("target_dist", m_target_dist, 1.0);
        nh_planner.param("goal_dist_tolerance", m_goal_dist_tolerance, 0.1);
        nh_planner.param("goal_yaw_tolerance", m_goal_yaw_tolerance, 0.1);
        nh_planner.param("scan_topic", m_scan_topic, std::string("/scan"));
        nh_planner.param("base_frame_id", m_base_frame_id, std::string("base_link"));
        nh_planner.param("odom_frame_id", m_odom_frame_id, std::string("odom"));

        ROS_WARN("%s initialized", name.c_str());
        ROS_WARN("pre_n: %d", pre_n);
        ROS_WARN("Kp: %f", Kp);
        ROS_WARN("Ki: %f", Ki);
        ROS_WARN("Kd: %f", Kd);
        ROS_WARN("max_vel_trans: %f", m_max_vel_trans);
        ROS_WARN("max_vel_rot: %f", m_max_vel_rot);
        ROS_WARN("acc_scale_trans: %f", m_acc_scale_trans);
        ROS_WARN("acc_scale_rot: %f", m_acc_scale_rot);
        ROS_WARN("goal_dist_tolerance: %f", m_goal_dist_tolerance);
        ROS_WARN("goal_yaw_tolerance: %f", m_goal_yaw_tolerance);
        ROS_WARN("scan_topic: %s", m_scan_topic.c_str());
        ROS_WARN("base_frame_id: %s", m_base_frame_id.c_str());
        ROS_WARN("odom_frame_id: %s", m_odom_frame_id.c_str());
    }

    void MyPlanner::reconfigureCB(my_planner::MyPlannerConfig& config, uint32_t level)
    {
        if(!setup_)
        {
            //初次初始化，保存默认参数
            default_config_ = config;
            setup_ = true;
            return;
        }
        else if(setup_ && config.restore_defaults)
        {
            config = default_config_;
            ROS_WARN("Restore default parameters");
            return;
        }
        pre_n = config.pre_n;
        trans_x_factor = config.trans_x_factor;
        trans_y_factor = config.trans_y_factor;
        adjust_r_factor = config.adjust_r_factor;
        Kp = config.Kp;
        Ki = config.Ki;
        Kd = config.Kd;
        m_target_dist = config.target_dist;
        m_max_vel_trans = config.max_vel_trans;
        m_max_vel_rot = config.max_vel_rot;
        m_acc_scale_trans = config.acc_scale_trans;
        m_acc_scale_rot = config.acc_scale_rot;
        m_goal_dist_tolerance = config.goal_dist_tolerance;
        m_goal_yaw_tolerance = config.goal_yaw_tolerance;

        ROS_WARN("Reconfigure Request: Kp=%f, Ki=%f, Kd=%f, max_vel_trans=%f, max_vel_rot=%f, acc_scale_trans=%f, acc_scale_rot=%f, goal_dist_tolerance=%f, goal_yaw_tolerance=%f", Kp, Ki, Kd, m_max_vel_trans, m_max_vel_rot, m_acc_scale_trans, m_acc_scale_rot, m_goal_dist_tolerance, m_goal_yaw_tolerance);
    }

    std::vector<geometry_msgs::PoseStamped> global_plan_;
    bool goal_reached_ = false;
    int target_index_ = 0;
    bool pose_adjusting_=  false;

    bool MyPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
    { 
        ROS_WARN("Setting plan");
        global_plan_ = plan;
        target_index_ = 0;
        pose_adjusting_ = false;
        goal_reached_ = false;
        angular_error = 0.0;
        error_sum = 0.0;
        last_error = 0.0;
        error_diff = 0.0;
        output = 0.0;
        return true;
    }

    bool MyPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
    {
        ROS_WARN("Computing velocity commands");
        costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap(); // 获取costmap
        if(!costmap)
        {
            ROS_ERROR("Costmap is not available");
            // return false;
        }
        unsigned char* map = costmap->getCharMap();
        unsigned int size_x = costmap->getSizeInCellsX();
        unsigned int size_y = costmap->getSizeInCellsY();
        double resolution = costmap->getResolution();

        // cv::Mat map_image(size_y, size_x, CV_8UC3, cv::Scalar(128,128,128)); // 代价地图绘制数据
        // for(unsigned int y=0; y<size_y; y++)
        // {
        //     for(unsigned int x=0; x<size_x; x++)
        //     {
        //        int map_index = y*size_x + x;
        //        unsigned char cost = map[map_index]; // 获取该点的代价值
        //     //    cv::Vec3b& color = map_image.at<cv::Vec3b>(map_index);

        //        if(cost == 0)
        //        {
        //            color = cv::Vec3b(128,128,128); // 灰色表示该点可行走
        //        }
        //        else if(cost == 254) //障碍物
        //        {
        //            color = cv::Vec3b(0,0,0); // 黑色表示该点不可行走
        //        }
        //        else if(cost == 253) //禁行区
        //        {
        //            color =  cv::Vec3b(255,255,0); // 浅蓝
        //        }
        //        else
        //        {
        //         unsigned char blue = 255 - cost;
        //         unsigned char red = cost;
        //         color = cv::Vec3b(blue,0,red); // 红色表示代价值越小，颜色越深
        //        }
        //     }
        // }

        //获取全局路径
        for(int i=0; i<global_plan_.size(); i++)
        {
            geometry_msgs::PoseStamped pose_odom;
            global_plan_[i].header.stamp = ros::Time(0);
            tf_buffer_->transform(global_plan_[i], pose_odom, m_odom_frame_id, ros::Duration(1.0));
            double odom_x = pose_odom.pose.position.x; 
            double odom_y = pose_odom.pose.position.y; 

            double origin_x = costmap->getOriginX(); //局部代价地图原点
            double origin_y = costmap->getOriginY();
            double local_x = odom_x - origin_x; // 距离值 单位 ：米 
            double local_y = odom_y - origin_y;
            int x = local_x / resolution; // 距离值 单位 ：cell
            int y = local_y / resolution;
            // cv::circle(map_image, cv::Point(x, y), 0, cv::Scalar(255,0,255)); // 在代价地图中画出路径点

            if(i >= target_index_ && i < (target_index_+pre_n))//检测跟踪点的后N个点 看是否有障碍物 
            {
                // cv::circle(map_image, cv::Point(x, y), 0, cv::Scalar(0,255,255)); // 在代价地图中画出检测点
                int map_index = y*size_x + x;
                unsigned char cost = map[map_index]; // 获取该点的代价值
                if(cost >= 253) //障碍物
                {
                    ROS_WARN("检测到障碍物");
                    return false;
                }

            }
        }

        // map_image.at<cv::Vec3b>(size_y/2, size_x/2) = cv::Vec3b(0,255,0); // 机器人位置


        // cv::Mat flipped_image(size_x, size_y, CV_8UC3, cv::Scalar(128,128,128));
        // for(unsigned int y=0; y<size_y; y++)
        // {
        //     for(unsigned int x=0; x<size_x; x++)
        //     {
        //         cv::Vec3b& pixel = map_image.at<cv::Vec3b>(y,x); //代价地图对角线翻转
        //         flipped_image.at<cv::Vec3b>((size_x-1-x),(size_y-1-y)) = pixel; //x 、y 轴翻转
        //     }
        // }

        // map_image = flipped_image; // 绘制代价地图
        // cv::namedWindow("Costmap");
        // cv::resize(map_image, map_image, cv::Size(size_y*5, size_x*5),0,0,cv::INTER_NEAREST);
        // cv::resizeWindow("Costmap", size_y*5, size_x*5);
        // cv::imshow("Costmap", map_image);

        int final_index = global_plan_.size()-1;
        geometry_msgs::PoseStamped pose_final;
        tf_buffer_->transform(global_plan_[final_index], pose_final, m_base_frame_id, ros::Duration(1.0));
        if(pose_adjusting_==false)
        {
            double dx = pose_final.pose.position.x;
            double dy = pose_final.pose.position.y;
            double dist = sqrt(dx*dx + dy*dy);
            if(dist < m_goal_dist_tolerance)//与目标点距离小于 0.05 米，认为已经到达目标点
            {
                pose_adjusting_ = true;
            }
        }
        if(pose_adjusting_==true)
        {
            double final_yaw = tf2::getYaw(pose_final.pose.orientation);
            ROS_WARN("调整目标姿态,final_yaw=%f", final_yaw);
            cmd_vel.linear.x = pose_final.pose.position.x*trans_x_factor;
            cmd_vel.linear.y = pose_final.pose.position.y*trans_y_factor;
            cmd_vel.angular.z = final_yaw* adjust_r_factor;

            if(abs(final_yaw) < m_goal_yaw_tolerance)
            {
                goal_reached_ = true;
                ROS_WARN("到达目标点");
                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = 0;
            }
            return true;
        }

        geometry_msgs::PoseStamped target_pose;
        for(int i = target_index_; i < global_plan_.size(); i++)
        {
            geometry_msgs::PoseStamped pose_base;
            global_plan_[i].header.stamp = ros::Time(0);
            //路径点转换到机器人坐标系
            try
            {
                // tf_listener_->waitForTransform(m_base_frame_id, global_plan_[i].header.frame_id, global_plan_[i].header.stamp, ros::Duration(1.0));
                tf_buffer_->transform(global_plan_[i], pose_base, m_base_frame_id, ros::Duration(1.0));
            }
            catch(tf2::TransformException& e)
            {
                ROS_ERROR("%s", e.what());
                continue;
            }
            double dx = pose_base.pose.position.x;
            double dy = pose_base.pose.position.y;
            double dist = sqrt(dx*dx + dy*dy);

            if(dist > m_target_dist)
            {
                target_pose = pose_base;
                target_index_ = i;
                ROS_WARN("选择第%d 个点作为目标点，位置(%f, %f)", i, target_pose.pose.position.x, target_pose.pose.position.y);
                break;
            }

            if(i == global_plan_.size()-1)
            {
                target_pose = pose_base;
                target_index_ = i;
                break;
            }
        }
        //路线追踪计算速度指令
        cmd_vel.linear.x = target_pose.pose.position.x*trans_x_factor;
        cmd_vel.linear.y = target_pose.pose.position.y*trans_y_factor;
        // 误差
        angular_error = target_pose.pose.position.y; 
        // 误差积分
        error_sum += angular_error;
        if(error_sum > 10000 || error_sum < -10000) //   防止积分溢出
        {
            error_sum = 0;
        }
        // 误差微分
        error_diff = angular_error - last_error;
        //PID控制
        output = Kp * angular_error + Ki * error_sum + Kd * error_diff;
        ROS_WARN("Kp=%f, Ki=%f, Kd=%f, angular_error=%f, error_sum=%f, error_diff=%f, output=%f", Kp, Ki, Kd, angular_error, error_sum, error_diff, output);
        // 限制输出
        cmd_vel.angular.z = output;
        ROS_WARN("angular_error=%f, output=%f", angular_error, output);
        // 保存上一次的误差
        last_error = angular_error;


        // 》》》》》》》》》》》》》》》》目标朝向跟踪控制：
        // geometry_msgs::PoseStamped pose_map, pose_base;
        // pose_map.pose.position.x = 1.0; //    目标点在地图坐标系下的坐标
        // pose_map.pose.position.y = -1.5;
        // pose_map.pose.orientation.w = 1.0;
        // pose_map.header.frame_id = "map";
        // pose_map.header.stamp = ros::Time(0);
        // try
        // {
        //     tf_buffer_->transform(pose_map, pose_base, m_base_frame_id, ros::Duration(1.0)); //   目标点转换到机器人坐标系
        // }
        // catch(tf2::TransformException& e)
        // {
        //     ROS_ERROR("%s", e.what());
        //     return false;
        // }
        // cmd_vel.angular.z = pose_base.pose.position.y*1.0;
        // 《《《《《《《《《《《《《《《《《《《《《《

        // 》》》》》》》》》》》》》》》》》》》》》》》》》》》》绘制全局路径
        // cv::Mat plan_image(600, 600, CV_8UC3, cv::Scalar(0,0,0));
        // for(int i=0; i<global_plan_.size(); i++)
        // {
        //     //坐标换算
        //     geometry_msgs::PoseStamped pose_base;
        //     global_plan_[i].header.stamp = ros::Time(0);
        //     try
        //     {
        //         // tf_listener_->waitForTransform(m_base_frame_id, global_plan_[i].header.frame_id, global_plan_[i].header.stamp, ros::Duration(1.0));
        //         tf_buffer_->transform(global_plan_[i], pose_base, m_base_frame_id, ros::Duration(1.0));
        //     }
        //     catch(tf2::TransformException& e)
        //     {
        //         ROS_ERROR("%s", e.what());
        //         continue;
        //     }
     
        //     // int cv_x = 300 - pose_base.pose.position.y*100; // ros坐标系下y轴向下为正，cv坐标系下y轴向上为正
        //     // int cv_y = 300 - pose_base.pose.position.x*100; // ros坐标系下x轴向右为正，cv坐标系下x轴向左为正
        //     // cv::circle(plan_image, cv::Point(cv_x, cv_y), 1, cv::Scalar(255,0,255));
        // }
        // cv::circle(plan_image, cv::Point(300, 300), 15, cv::Scalar(0,255,0)); // 起点
        // cv::line (plan_image, cv::Point(65, 300), cv::Point(510, 300), cv::Scalar(0,255,0), 1); // x轴
        // cv::line (plan_image, cv::Point(300, 45), cv::Point(300, 555), cv::Scalar(0,255,0), 1); // y轴
        // // cv::namedWindow("Plan");
        // // cv::imshow("Plan", plan_image);
        // cv::waitKey(1);
        // 《《《《《《《《《《《《《《《《《《《《《《《《《《《《《《《《《《《


        return true;
    }
    bool MyPlanner::isGoalReached()
    {
        return goal_reached_;
    }

}// namespace my_planner