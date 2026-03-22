#include <ros/ros.h>
#include <ros/master.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include <cctype>
#include <limits>
#include <set>
#include <string>
#include <sstream>
#include <memory>

std::vector<int> detectRobotIdsFromTopics(
    const std::string& namespace_prefix,
    const std::string& topic_suffix,
    double timeout_sec)
{
    const std::string prefix = "/" + namespace_prefix;
    const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(std::max(0.0, timeout_sec));
    const double settle_sec = timeout_sec > 0.0 ? std::min(0.6, timeout_sec) : 0.0;
    ros::WallRate poll_rate(5.0);
    std::set<int> best_ids;
    ros::WallTime last_change = ros::WallTime::now();

    while (ros::ok()) {
        std::set<int> found_ids;
        ros::master::V_TopicInfo topics;
        if (ros::master::getTopics(topics)) {
            for (const auto& topic_info : topics) {
                const std::string& topic_name = topic_info.name;
                if (topic_name.rfind(prefix, 0) != 0) {
                    continue;
                }

                std::size_t id_begin = prefix.size();
                std::size_t id_end = id_begin;
                while (id_end < topic_name.size() &&
                       std::isdigit(static_cast<unsigned char>(topic_name[id_end]))) {
                    ++id_end;
                }
                if (id_end == id_begin || id_end >= topic_name.size() || topic_name[id_end] != '/') {
                    continue;
                }
                if (!topic_suffix.empty() &&
                    topic_name.compare(id_end, topic_suffix.size(), topic_suffix) != 0) {
                    continue;
                }

                const int robot_id = std::stoi(topic_name.substr(id_begin, id_end - id_begin));
                if (robot_id > 0) {
                    found_ids.insert(robot_id);
                }
            }
        }

        const bool should_update_best =
            !found_ids.empty() &&
            (found_ids.size() > best_ids.size() ||
             (found_ids.size() == best_ids.size() && found_ids != best_ids));
        if (should_update_best) {
            best_ids = found_ids;
            last_change = ros::WallTime::now();
        }

        const ros::WallTime now = ros::WallTime::now();
        if (!best_ids.empty()) {
            const bool settled = settle_sec <= 0.0 || (now - last_change).toSec() >= settle_sec;
            if (settled || now >= deadline) {
                return std::vector<int>(best_ids.begin(), best_ids.end());
            }
        }
        if (timeout_sec <= 0.0 || now >= deadline) {
            break;
        }
        poll_rate.sleep();
    }

    return std::vector<int>(best_ids.begin(), best_ids.end());
}

bool resolveRobotConfig(ros::NodeHandle& nh, int& robot_num, std::vector<int>& robot_ids)
{
    bool auto_detect_robots = true;
    double robot_detect_timeout = 8.0;
    std::string robot_namespace_prefix = "robot";
    std::string robot_detect_topic_suffix = "/odom";
    nh.param("auto_detect_robots", auto_detect_robots, auto_detect_robots);
    nh.param("robot_detect_timeout", robot_detect_timeout, robot_detect_timeout);
    nh.param("robot_namespace_prefix", robot_namespace_prefix, robot_namespace_prefix);
    nh.param("robot_detect_topic_suffix", robot_detect_topic_suffix, robot_detect_topic_suffix);

    if (auto_detect_robots) {
        const std::vector<int> detected_ids = detectRobotIdsFromTopics(
            robot_namespace_prefix,
            robot_detect_topic_suffix,
            robot_detect_timeout);
        if (detected_ids.empty()) {
            ROS_ERROR("[Plan Monitor] auto_detect_robots=true but no robots were detected within %.2fs on %s*/%s",
                      robot_detect_timeout,
                      robot_namespace_prefix.c_str(),
                      robot_detect_topic_suffix.c_str());
            return false;
        }

        robot_ids = detected_ids;
        robot_num = static_cast<int>(robot_ids.size());
        ROS_INFO("[Plan Monitor] Auto-detected %d robots from topic suffix %s",
                 robot_num, robot_detect_topic_suffix.c_str());
    } else {
        bool has_num_param = nh.getParam("num_robots", robot_num);
        nh.getParam("robot_ids", robot_ids);

        if (has_num_param && robot_num > 0) {
            if (robot_ids.size() != static_cast<std::size_t>(robot_num)) {
                robot_ids.clear();
                robot_ids.reserve(robot_num);
                for (int i = 1; i <= robot_num; ++i) {
                    robot_ids.push_back(i);
                }
                ROS_WARN("[Plan Monitor] robot_ids mismatched configured num_robots=%d, regenerate sequential IDs 1..%d",
                         robot_num, robot_num);
            }
        }
    }

    if (robot_num <= 0 && !robot_ids.empty()) {
        robot_num = static_cast<int>(robot_ids.size());
    }
    if (robot_ids.empty() && robot_num > 0) {
        robot_ids.reserve(robot_num);
        for (int i = 1; i <= robot_num; ++i) {
            robot_ids.push_back(i);
        }
        ROS_WARN("[Plan Monitor] robot_ids missing, fallback to sequential IDs 1..%d", robot_num);
    }
    if (robot_num != static_cast<int>(robot_ids.size())) {
        ROS_WARN("[Plan Monitor] num_robots=%d mismatches robot_ids size=%zu, use robot_ids size",
                 robot_num, robot_ids.size());
        robot_num = static_cast<int>(robot_ids.size());
    }
    if (robot_num <= 0 || robot_ids.empty()) {
        ROS_ERROR("[Plan Monitor] No valid robot set found, disable monitor");
        return false;
    }

    nh.setParam("/num_robots", robot_num);
    nh.setParam("/robot_ids", robot_ids);
    nh.setParam("num_robots", robot_num);
    nh.setParam("robot_ids", robot_ids);
    return true;
}

bool getShapeTakeoverStopPlanning(ros::NodeHandle& nh, std::set<int>& active_robot_ids)
{
    active_robot_ids.clear();
    std::vector<int> active_ids_param;
    nh.getParam("/shape_assembly/active_robot_ids", active_ids_param);
    for (const int id : active_ids_param) {
        if (id > 0) {
            active_robot_ids.insert(id);
        }
    }

    bool stop_path_planning = false;
    nh.param("/shape_assembly/stop_path_planning", stop_path_planning, false);
    return stop_path_planning;
}

std::string makeRobotNamespace(const std::string& robot_namespace_prefix, const int robot_id)
{
    return "/" + robot_namespace_prefix + std::to_string(robot_id);
}

std::string makeRobotTopic(const std::string& robot_namespace_prefix,
                           const int robot_id,
                           const std::string& topic_suffix)
{
    std::string suffix = topic_suffix;
    if (!suffix.empty() && suffix.front() != '/') {
        suffix = "/" + suffix;
    }
    return makeRobotNamespace(robot_namespace_prefix, robot_id) + suffix;
}

class GlobalPlanMonitor {
public:
    GlobalPlanMonitor(int idx,
                      const std::string& robot_namespace_prefix,
                      const std::string& robot_odom_topic_suffix)
        : nh_("~"), gather_start_(0) {
        // 初始化时间戳为0值（无效时间）
        last_received_ = ros::Time(0);
        idx_ = idx;
        // 构建话题名称
        const std::string robot_namespace = makeRobotNamespace(robot_namespace_prefix, idx);
        plan_topic_ = makeRobotTopic(robot_namespace_prefix, idx, "move_base/GraphPlanner/plan");
        signal_topic_ = "/gather_signal";
        gather_start_topic_ = "/gather_started";
        mvstate_topic_ = makeRobotTopic(robot_namespace_prefix, idx, "mv_state");
        result_topic_ = makeRobotTopic(robot_namespace_prefix, idx, "move_base/result");
        goal_topic_ = makeRobotTopic(robot_namespace_prefix, idx, "move_base/current_goal");
        odom_topic_ = makeRobotTopic(robot_namespace_prefix, idx, robot_odom_topic_suffix);
        // 初始化发布订阅
        plan_sub_ = nh_.subscribe(plan_topic_, 1, &GlobalPlanMonitor::planCallback, this);
        gather_start_sub_ = nh_.subscribe(gather_start_topic_, 1, &GlobalPlanMonitor::gatherStartCallback, this);
        mvstate_sub_ = nh_.subscribe(mvstate_topic_, 1, &GlobalPlanMonitor::mvstateCallback, this);
        result_sub_ = nh_.subscribe(result_topic_, 1, &GlobalPlanMonitor::resultCallback, this);
        goal_sub_ = nh_.subscribe(goal_topic_, 1, &GlobalPlanMonitor::goalCallback, this);
        odom_sub_ = nh_.subscribe(odom_topic_, 1, &GlobalPlanMonitor::odomCallback, this);
        // 设置定时检测（1Hz）
        // timer_ = nh_.createTimer(ros::Duration(1.0), &GlobalPlanMonitor::checkTimeout, this);
        plan_status_=IDLE;

        ROS_INFO_STREAM("Monitoring: " << plan_topic_);
        ROS_INFO_STREAM("Signal topic: " << signal_topic_);
        ROS_INFO_STREAM("Gather start topic: " << gather_start_topic_);
        ROS_INFO_STREAM("Mv state topic: " << mvstate_topic_);
        ROS_INFO_STREAM("Result topic: " << result_topic_);
        ROS_INFO_STREAM("Goal topic: " << goal_topic_);
        ROS_INFO_STREAM("Odom topic: " << odom_topic_);
    }

    enum PlanStatus {
        IDLE = 0,
        ACTIVE = 1,
        TIMEOUT = 2
    };

    enum MvStates {
        NAVIGATING = 1,   // 对应ACTIVE
        PREEMPTED  = 3,   // 对应CANCELLED
        SUCCEEDED  = 4,
        ABORTED    = 5,   // 主要失败状态
    };

    bool isNavigating() {
        return mv_state_ == NAVIGATING;
    }

    bool isGathering() const {
        return gather_start_ == 1;
    }

    double get_plan_length()
    {
        return plan_length_;
    }

    bool hasCurrentGoal() const
    {
        return has_current_goal_;
    }

    geometry_msgs::PoseStamped getCurrentGoal() const
    {
        return current_goal_;
    }

    bool hasRobotPose() const
    {
        return has_robot_pose_;
    }

    geometry_msgs::PoseStamped getRobotPose() const
    {
        return robot_pose_;
    }

    bool consumeAborted()
    {
        if (aborted_) {
            aborted_ = false;
            return true;
        }
        return false;
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber plan_sub_;
    ros::Subscriber gather_start_sub_;
    ros::Subscriber mvstate_sub_;
    ros::Subscriber result_sub_;
    ros::Subscriber goal_sub_;
    ros::Subscriber odom_sub_;
    
    ros::Timer timer_;
    int idx_;
    std::string plan_topic_;
    std::string signal_topic_;
    std::string gather_start_topic_;
    std::string mvstate_topic_;
    std::string result_topic_;
    std::string goal_topic_;
    std::string odom_topic_;
    ros::Time last_received_;
    int gather_start_;
    PlanStatus   plan_status_;
    MvStates mv_state_;
    double plan_length_ = 0;
    bool aborted_ = false;
    geometry_msgs::PoseStamped current_goal_;
    bool has_current_goal_ = false;
    geometry_msgs::PoseStamped robot_pose_;
    bool has_robot_pose_ = false;

    double calculatePathLength(const nav_msgs::Path& path) {
        if (path.poses.empty()) {
            return 0.0;
        }

        double length = 0.0;
        geometry_msgs::PoseStamped prev_pose = path.poses.front();

        for (size_t i = 1; i < path.poses.size(); ++i) {
            const geometry_msgs::PoseStamped& current_pose = path.poses[i];
            double dx = current_pose.pose.position.x - prev_pose.pose.position.x;
            double dy = current_pose.pose.position.y - prev_pose.pose.position.y;
            length += std::sqrt(dx * dx + dy * dy);
            prev_pose = current_pose;
        }

        return length;
    }

    void planCallback(const nav_msgs::Path::ConstPtr& msg) {
        bool is_valid = !msg->poses.empty();
        double length = calculatePathLength(*msg);
        ROS_INFO_STREAM( "Path"<<idx_<< " status: " << (is_valid ? "Valid" : "Empty")<< "Length: " << length);
        
        // 只在路径有效时更新时间戳
        if (is_valid) {
            last_received_ = ros::Time::now();
            ROS_DEBUG_STREAM("Updating valid path timestamp");
            plan_length_ = length;
        }

    }

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        current_goal_ = *msg;
        has_current_goal_ = true;
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        robot_pose_.header = msg->header;
        robot_pose_.pose = msg->pose.pose;
        has_robot_pose_ = true;
    }

    void gatherStartCallback(const std_msgs::UInt8::ConstPtr& msg) {
        gather_start_ = msg->data;
        // ROS_INFO_STREAM("Gathering: " << (gather_start_ ? "YES" : "NO"));
        // 采集结束时重置时间戳
        if (gather_start_== 0) 
        { //聚集停止
            plan_status_ = PlanStatus::IDLE;
        }
        if(gather_start_ == 1) //聚集开始
        {
            plan_status_ = PlanStatus::ACTIVE;
        }
        last_received_ = ros::Time(0);
    }

    void mvstateCallback(const std_msgs::UInt8::ConstPtr& msg) {
        switch(msg->data) {
        case 1:
            {mv_state_ = NAVIGATING;
            break;}
        case 3:
            {mv_state_ = PREEMPTED;
            break;}
        case 4:
            {mv_state_ = SUCCEEDED; 
            break;}
        case 5:
            {
            mv_state_ = ABORTED;
            aborted_ = true;
            ROS_ERROR_STREAM("Mv "<<idx_<<" ABORTED");
            break;
            }
            
        default:
            mv_state_ = NAVIGATING; // 默认处理
        }
        ROS_INFO_STREAM("Mv "<<idx_<<" state: " << static_cast<int>(mv_state_));
    }

    void resultCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg) {
        const uint8_t status = msg->status.status;
        switch (status) {
        case 3:  // SUCCEEDED
            mv_state_ = SUCCEEDED;
            break;
        case 2:  // PREEMPTED
            mv_state_ = PREEMPTED;
            break;
        case 4:  // ABORTED
        case 5:  // REJECTED
        case 9:  // LOST
            mv_state_ = ABORTED;
            aborted_ = true;
            ROS_ERROR_STREAM("Move base result for robot " << idx_
                             << " failed with status=" << static_cast<int>(status)
                             << " text=" << msg->status.text);
            break;
        default:
            break;
        }
    }

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "global_plan_monitor");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    int robot_num = 0;
    std::vector<int> robot_ids;
    if (!resolveRobotConfig(nh, robot_num, robot_ids)) {
        return 1;
    }
    
    ROS_WARN_STREAM("Monitoring " << robot_num << " robots");
    std::ostringstream ids_oss;
    for (std::size_t i = 0; i < robot_ids.size(); ++i) {
        if (i > 0) {
            ids_oss << ",";
        }
        ids_oss << robot_ids[i];
    }
    ROS_WARN_STREAM("Robot ids: [" << ids_oss.str() << "]");
    std::string signal_topic_ = "/gather_signal";
    std::string reason_topic_ = "/gather_replan_event";
    std::string robot_namespace_prefix = "robot";
    std::string robot_odom_topic_suffix = "/odom";
    nh.param("robot_namespace_prefix", robot_namespace_prefix, robot_namespace_prefix);
    nh.param("robot_odom_topic_suffix", robot_odom_topic_suffix, robot_odom_topic_suffix);
    nh.param("robot_detect_topic_suffix", robot_odom_topic_suffix, robot_odom_topic_suffix);
    ros::Publisher signal_pub = nh.advertise<std_msgs::UInt8>(signal_topic_, 1);
    ros::Publisher reason_pub = nh.advertise<std_msgs::String>(reason_topic_, 10);
    //路径监视器
    std::vector<std::shared_ptr<GlobalPlanMonitor>> monitors;

    // replan config
    std::string replan_mode_str;
    pnh.param("replan_mode", replan_mode_str, std::string("event")); // off | event | periodic | hybrid
    double periodic_interval = 5.0;
    double cooldown = 2.0;
    double per_robot_cooldown = 1.5;
    double stable_len_epsilon = 0.02;
    int stable_len_count_thres = 15;
    double stable_len_min = 0.0;
    double stable_len_max = 1.0;
    double max_len_growth_ratio = 1.2;
    double long_path_replan_min_length = 0.0;
    double format_radius = 0.3;
    bool enable_stable_len_replan = false;
    bool use_goal_occupied_peer_fallback = true;
    double goal_occupied_radius = 0.35;
    double loop_hz = 10.0;
    pnh.param("robot_namespace_prefix", robot_namespace_prefix, robot_namespace_prefix);
    pnh.param("robot_odom_topic_suffix", robot_odom_topic_suffix, robot_odom_topic_suffix);
    pnh.param("periodic_interval", periodic_interval, periodic_interval);
    pnh.param("cooldown", cooldown, cooldown);
    pnh.param("per_robot_cooldown", per_robot_cooldown, per_robot_cooldown);
    pnh.param("stable_len_epsilon", stable_len_epsilon, stable_len_epsilon);
    pnh.param("stable_len_count", stable_len_count_thres, stable_len_count_thres);
    pnh.param("stable_len_min", stable_len_min, stable_len_min);
    pnh.param("stable_len_max", stable_len_max, stable_len_max);
    pnh.param("max_len_growth_ratio", max_len_growth_ratio, max_len_growth_ratio);
    pnh.param("long_path_replan_min_length", long_path_replan_min_length, long_path_replan_min_length);
    pnh.param("format_radius", format_radius, format_radius);
    pnh.param("enable_stable_len_replan", enable_stable_len_replan, enable_stable_len_replan);
    pnh.param("use_goal_occupied_peer_fallback", use_goal_occupied_peer_fallback, use_goal_occupied_peer_fallback);
    pnh.param("goal_occupied_radius", goal_occupied_radius, goal_occupied_radius);
    pnh.param("loop_hz", loop_hz, loop_hz);

    auto modeIs = [&](const std::string& value) {
        return replan_mode_str == value;
    };
    bool mode_event = modeIs("event") || modeIs("hybrid");
    bool mode_periodic = modeIs("periodic") || modeIs("hybrid");
    bool mode_off = modeIs("off");
    if (mode_off) {
        mode_event = false;
        mode_periodic = false;
    }

    double perimeter = 2* 3.14159265358979323846 * format_radius;
    for(int i = 0; i < robot_num; ++i)
    {
        monitors.push_back(std::make_shared<GlobalPlanMonitor>(
            robot_ids[i],
            robot_namespace_prefix,
            robot_odom_topic_suffix));
    }
    
    goal_occupied_radius = std::max(0.0, goal_occupied_radius);
    std::vector<int> len_counts(robot_num,0);
    std::vector<double> len_record(robot_num,0);
    std::vector<double> last_plan_length_for_change(robot_num, 0.0);
    std::vector<bool> last_plan_length_valid(robot_num, false);
    std::vector<ros::Time> last_robot_replan_time(robot_num, ros::Time(0));
    ros::Time last_replan_time(0);
    ros::Time last_periodic_time(0);
    bool last_stop_path_planning = false;

    auto publishReplan = [&](uint8_t signal, const std::string& reason) {
        std_msgs::UInt8 s;
        s.data = signal;
        signal_pub.publish(s);

        std_msgs::String info;
        info.data = reason;
        reason_pub.publish(info);
        last_replan_time = ros::Time::now();
    };

    ros::Rate rate(loop_hz);

    auto goalOccupiedByPeer = [&](int robot_index, int* occupied_by_peer_idx, double* occupied_distance) {
        if (occupied_by_peer_idx) {
            *occupied_by_peer_idx = -1;
        }
        if (occupied_distance) {
            *occupied_distance = std::numeric_limits<double>::infinity();
        }
        if (robot_index < 0 || robot_index >= static_cast<int>(monitors.size())) {
            return false;
        }
        if (goal_occupied_radius <= 0.0 || !monitors[robot_index]->hasCurrentGoal()) {
            return false;
        }

        const geometry_msgs::PoseStamped goal = monitors[robot_index]->getCurrentGoal();
        const double gx = goal.pose.position.x;
        const double gy = goal.pose.position.y;
        double best_distance = std::numeric_limits<double>::infinity();
        int best_peer = -1;

        for (int j = 0; j < robot_num; ++j) {
            if (j == robot_index || !monitors[j]->hasRobotPose()) {
                continue;
            }
            const geometry_msgs::PoseStamped pose = monitors[j]->getRobotPose();
            const double dx = pose.pose.position.x - gx;
            const double dy = pose.pose.position.y - gy;
            const double distance = std::sqrt(dx * dx + dy * dy);
            if (distance <= goal_occupied_radius && distance < best_distance) {
                best_distance = distance;
                best_peer = j;
            }
        }

        if (best_peer < 0) {
            return false;
        }
        if (occupied_by_peer_idx) {
            *occupied_by_peer_idx = best_peer;
        }
        if (occupied_distance) {
            *occupied_distance = best_distance;
        }
        return true;
    };

    while(ros::ok())
    {
        ros::spinOnce();
        std::set<int> shape_active_robot_ids;
        const bool stop_path_planning =
            getShapeTakeoverStopPlanning(nh, shape_active_robot_ids);
        if (stop_path_planning != last_stop_path_planning) {
            std::ostringstream oss;
            for (auto it = shape_active_robot_ids.begin(); it != shape_active_robot_ids.end(); ++it) {
                if (it != shape_active_robot_ids.begin()) {
                    oss << ",";
                }
                oss << *it;
            }
            if (stop_path_planning) {
                ROS_WARN_STREAM("Shape takeover requested global replanning stop. active_ids=[" << oss.str() << "]");
            } else {
                ROS_INFO("Shape takeover cleared, resume path replanning.");
            }
            last_replan_time = ros::Time::now();
            last_periodic_time = ros::Time::now();
        }
        last_stop_path_planning = stop_path_planning;

        if (stop_path_planning) {
            rate.sleep();
            continue;
        }

        ros::Time now = ros::Time::now();
        bool gather_active = (!monitors.empty()) ? monitors[0]->isGathering() : false;
        int inactive_robot_count = 0;
        for (const int robot_id : robot_ids) {
            if (shape_active_robot_ids.find(robot_id) == shape_active_robot_ids.end()) {
                ++inactive_robot_count;
            }
        }

        // periodic replan
        if (mode_periodic && gather_active && inactive_robot_count > 0) {
            if ((now - last_periodic_time).toSec() >= periodic_interval &&
                (now - last_replan_time).toSec() >= cooldown) {
                publishReplan(2, "PERIODIC interval=" + std::to_string(periodic_interval));
                last_periodic_time = now;
                ROS_WARN_STREAM("Periodic replan triggered");
            }
        }

        for(int i =0; i<robot_num; i++)
        {
            if (shape_active_robot_ids.find(robot_ids[i]) != shape_active_robot_ids.end()) {
                continue;
            }
            double plan_length = monitors[i]->get_plan_length();
            if (mode_event && gather_active) {
                if(monitors[i]->consumeAborted())
                {
                    if ((now - last_replan_time).toSec() >= cooldown) {
                        int peer_idx = -1;
                        double peer_distance = std::numeric_limits<double>::infinity();
                        if (use_goal_occupied_peer_fallback &&
                            goalOccupiedByPeer(i, &peer_idx, &peer_distance) &&
                            (now - last_robot_replan_time[i]).toSec() >= per_robot_cooldown) {
                            publishReplan(
                                static_cast<uint8_t>(i + 11),
                                "GOAL_OCCUPIED robot=" + std::to_string(robot_ids[i]) +
                                " peer=" + std::to_string(robot_ids[peer_idx]) +
                                " dist=" + std::to_string(peer_distance));
                            last_robot_replan_time[i] = now;
                            ROS_WARN_STREAM("Robot " << robot_ids[i]
                                            << " aborted because goal is occupied by robot "
                                            << robot_ids[peer_idx]
                                            << ", send per-robot goal perturbation");
                        } else {
                            publishReplan(2, "ABORTED robot=" + std::to_string(robot_ids[i]));
                            ROS_ERROR_STREAM("Mv " << robot_ids[i] << " ABORTED, trigger gather-point replan");
                        }
                    }
                }

                if(enable_stable_len_replan &&
                   monitors[i]->isNavigating() &&
                   plan_length > stable_len_min && plan_length < stable_len_max)
                {
                    if(fabs(plan_length-len_record[i]) > stable_len_epsilon)
                    {
                        len_counts[i] = 0;
                        len_record[i] = plan_length;
                    }
                    else
                    {
                        len_counts[i]++;
                    }
                    if(len_counts[i] > stable_len_count_thres)
                    {
                        if ((now - last_robot_replan_time[i]).toSec() >= per_robot_cooldown) {
                            ROS_WARN_STREAM("Path length "<< i+1 <<" is too stable, REPLAN!");
                            publishReplan(static_cast<uint8_t>(i + 11),
                                "STABLE_LEN robot=" + std::to_string(robot_ids[i]) + " len=" + std::to_string(plan_length));
                            last_robot_replan_time[i] = now;
                            len_counts[i] = 0;
                        }
                    }
                }
                const double long_path_trigger_min_length =
                    std::max(perimeter, long_path_replan_min_length);
                if (monitors[i]->isNavigating() && plan_length > perimeter) {
                    if (last_plan_length_valid[i] &&
                        last_plan_length_for_change[i] > 1e-6 &&
                        plan_length > last_plan_length_for_change[i] * max_len_growth_ratio &&
                        plan_length >= long_path_trigger_min_length) {
                        if ((now - last_replan_time).toSec() >= cooldown) {
                            ROS_WARN_STREAM("Path length for robot " << robot_ids[i]
                                            << " grew from " << last_plan_length_for_change[i]
                                            << " to " << plan_length
                                            << " (trigger min length: " << long_path_trigger_min_length
                                            << "), REPLAN!");
                            publishReplan(
                                2,
                                "LONG_PATH robot=" + std::to_string(robot_ids[i]) +
                                " prev=" + std::to_string(last_plan_length_for_change[i]) +
                                " curr=" + std::to_string(plan_length) +
                                " min=" + std::to_string(long_path_trigger_min_length));
                        }
                    }
                    last_plan_length_for_change[i] = plan_length;
                    last_plan_length_valid[i] = true;
                } else if (plan_length <= 0.0) {
                    last_plan_length_valid[i] = false;
                }
            }
        }

        rate.sleep();
    }

    return 0;
}
