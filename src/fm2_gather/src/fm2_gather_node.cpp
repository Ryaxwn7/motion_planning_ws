#include <ros/ros.h>
#include <ros/master.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/UInt8.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <cctype>
#include <sstream>
#include <string>
#include <set>
#include <iomanip>
#include <array>
#include <map>
#include <random>
#include <cstdlib>
#include <cstdint>
#include <filesystem>
#include <boost/bind.hpp>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include "fastmarching/fm2/fastmarching2.hpp"
#include "fastmarching/ndgridmap/ndgridmap.hpp"
#include "fastmarching/io/gridplotter.hpp"
#include "fastmarching/io/gridwriter.hpp"
#include "fm2_gather/InitAndGoal.h"
#include "fm2_gather/dims.h"
#include "fm2_gather/map.h"
#include "fm2_gather/pathFM.h"
#include "fastmarching/fm2/gather.hpp"
#include "fm2_gather/move_base_controller.h"
#define GOAL_RAND_RANGE 0.1
// bool test_mode=false;

typedef std::vector<std::array<double, 2>> Path;
typedef std::vector<double> Path_v;
typedef std::vector<uint8_t> OccupancyMask;
std::vector<geometry_msgs::PoseStamped> robot_poses;
std::vector<std::array<float,2>> goals; //各机器人目标点
// std::vector<FastMarching2<nDGridMap<FMCell, 2>>> fm2_solvers;

namespace {
std::string getDebugOutputDir()
{
    namespace fs = std::filesystem;
    const char* custom = std::getenv("FM2_GATHER_DEBUG_DIR");
    fs::path base;
    if (custom != nullptr && custom[0] != '\0') {
        base = custom;
    } else {
        const char* ros_home = std::getenv("ROS_HOME");
        if (ros_home != nullptr && ros_home[0] != '\0') {
            base = fs::path(ros_home) / "fm2_gather_debug";
        } else {
            const char* home = std::getenv("HOME");
            if (home != nullptr && home[0] != '\0') {
                base = fs::path(home) / ".ros" / "fm2_gather_debug";
            } else {
                base = fs::path("/tmp") / "fm2_gather_debug";
            }
        }
    }
    std::error_code ec;
    fs::create_directories(base, ec);
    return base.string();
}

std::string makeDebugFilepath(const std::string& filename)
{
    namespace fs = std::filesystem;
    return (fs::path(getDebugOutputDir()) / filename).string();
}
}
fm2_gather::map occ;
int Min_idx;
std::array<int ,2> C_coord;
int num_robots; // 机器人数量
std::string robot_namespace_prefix = "robot";
int size_x_;
int size_y_;
int size_n_;
int ndims;
float resolution = -1;
std::array<int, 2> dimsize;
bool start_compute=false;
bool Map_init=false;
std::vector<int> robot_ids;
std::vector<double> robot_weights;
double weight_sum=0;
double dominantrate=0.02;
bool debug_on=true;
bool reverse =false;
bool pub_once=false;
bool publish_goal=true;
bool plot_on=true;
bool save_data=false;
std::vector<bool> robot_replan_flags; //用于指定需要重新规划路径的机器人id
bool replan_flag=false; //     用于判断是否需要重新规划路径
bool publish_center_goal_only=false;
std::string gather_center_topic = "/gather_center";
std::string external_center_goal_topic = "/shape_assembly/center_goal_cmd";
double external_center_sync_wait = 0.5;
bool external_center_pending = false;
std::array<float, 2> external_center_goal = {0.0f, 0.0f};
ros::Time external_center_dispatch_after;
bool use_combined_map = true;
std::string combined_map_topic = "/combined_map";
bool use_dynamic_obstacle_planning_grid = true;
double v_max = 1.0;
double velocity_alpha = 0.2;
double velocity_dmax = 0.5;
double robot_radius = 0.25;
int velocity_mode = 1;
double velocity_sigmoid_k = 1.5;
double velocity_sigmoid_b = 0.0;
double space_inflation_radius = 0.02;
double gather_radius = -1.0;
double goal_occupied_staging_radius = -1.0;
double goal_occupied_staging_margin = 0.25;
std::string base_map_topic = "/map";
bool use_dynamic_obstacle_velocity_layer = true;
std::string dynamic_obstacle_topic_suffix = "move_base/local_costmap/costmap";
int dynamic_obstacle_threshold = 80;
bool dynamic_obstacle_unknown_is_obstacle = false;
double dynamic_obstacle_timeout = 1.0;
double dynamic_obstacle_inflation_radius = 0.2;  // hard-core radius
double dynamic_obstacle_uncertainty_radius = 0.6; // soft uncertainty radius
double dynamic_obstacle_self_ignore_radius = 0.25;
int dynamic_obstacle_stride = 1;
int dynamic_obstacle_max_samples = 800;
bool use_peer_robot_uncertainty_layer = true;
double peer_robot_hard_radius = 0.2;
double peer_robot_uncertainty_radius = 0.6;
double peer_robot_speed_radius_gain = 0.3;
double peer_robot_uncertainty_max_radius = 1.2;
double peer_robot_speed_filter_alpha = 0.35;
bool dynamic_obstacle_ignore_swarm_robots = true;
double dynamic_obstacle_swarm_ignore_radius = 0.35;
bool publish_velocity_maps = true;
std::string velocity_map_topic_prefix = "/fm2_gather/velocity_map";
bool publish_arrival_time_maps = true;
std::string arrival_time_topic_prefix = "/fm2_gather/arrival_time";
std::vector<nav_msgs::OccupancyGrid> dynamic_obstacle_maps;
std::vector<bool> dynamic_obstacle_ready;
std::vector<ros::Time> dynamic_obstacle_stamp;
std::vector<std::array<double, 2>> robot_prev_positions;
std::vector<ros::Time> robot_prev_pose_stamp;
std::vector<double> robot_speed_estimates;
ros::Publisher velocity_map_base_pub;
std::vector<ros::Publisher> velocity_map_robot_pubs;
ros::Publisher arrival_time_sum_pub;
std::vector<ros::Publisher> arrival_time_robot_pubs;

std::string makeRobotNamespace(const int robot_id)
{
    return "/" + robot_namespace_prefix + std::to_string(robot_id);
}

std::string makeRobotTopic(const int robot_id, const std::string& topic_suffix)
{
    std::string suffix = topic_suffix;
    if (!suffix.empty() && suffix.front() != '/') {
        suffix = "/" + suffix;
    }
    return makeRobotNamespace(robot_id) + suffix;
}

struct UncertaintyKernelCell {
    int dx;
    int dy;
    double ratio;
};

std::map<long long, std::vector<UncertaintyKernelCell>> uncertainty_kernel_cache;
double uncertainty_kernel_resolution = -1.0;

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

bool resolveRobotConfig(ros::NodeHandle& nh, int& resolved_num, std::vector<int>& resolved_ids, std::vector<double>& resolved_weights)
{
    bool auto_detect_robots = true;
    double robot_detect_timeout = 8.0;
    std::string robot_detect_topic_suffix = "/odom";
    nh.param("auto_detect_robots", auto_detect_robots, auto_detect_robots);
    nh.param("robot_detect_timeout", robot_detect_timeout, robot_detect_timeout);
    nh.param("robot_namespace_prefix", robot_namespace_prefix, robot_namespace_prefix);
    nh.param("robot_detect_topic_suffix", robot_detect_topic_suffix, robot_detect_topic_suffix);

    nh.getParam("robot_weights", resolved_weights);

    if (auto_detect_robots) {
        const std::vector<int> detected_ids = detectRobotIdsFromTopics(
            robot_namespace_prefix,
            robot_detect_topic_suffix,
            robot_detect_timeout);
        if (detected_ids.empty()) {
            ROS_ERROR("[FM2 Gather] auto_detect_robots=true but no robots were detected within %.2fs on %s*/%s",
                      robot_detect_timeout,
                      robot_namespace_prefix.c_str(),
                      robot_detect_topic_suffix.c_str());
            return false;
        }

        resolved_ids = detected_ids;
        resolved_num = static_cast<int>(resolved_ids.size());
        ROS_INFO("[FM2 Gather] Auto-detected %d robots from topic suffix %s",
                 resolved_num, robot_detect_topic_suffix.c_str());
    } else {
        bool has_num_param = nh.getParam("num_robots", resolved_num);
        nh.getParam("robot_ids", resolved_ids);

        if (has_num_param && resolved_num > 0) {
            if (resolved_ids.size() != static_cast<std::size_t>(resolved_num)) {
                resolved_ids.clear();
                resolved_ids.reserve(resolved_num);
                for (int i = 1; i <= resolved_num; ++i) {
                    resolved_ids.push_back(i);
                }
                ROS_WARN("[FM2 Gather] robot_ids mismatched configured num_robots=%d, regenerate sequential IDs 1..%d",
                         resolved_num, resolved_num);
            }
        }
    }

    if (resolved_num <= 0 && !resolved_ids.empty()) {
        resolved_num = static_cast<int>(resolved_ids.size());
    }
    if (resolved_ids.empty() && resolved_num > 0) {
        resolved_ids.reserve(resolved_num);
        for (int i = 1; i <= resolved_num; ++i) {
            resolved_ids.push_back(i);
        }
        ROS_WARN("[FM2 Gather] robot_ids missing, fallback to sequential IDs 1..%d", resolved_num);
    }
    if (resolved_num != static_cast<int>(resolved_ids.size())) {
        ROS_WARN("[FM2 Gather] num_robots=%d mismatches robot_ids size=%zu, use robot_ids size",
                 resolved_num, resolved_ids.size());
        resolved_num = static_cast<int>(resolved_ids.size());
    }
    if (resolved_num <= 0 || resolved_ids.empty()) {
        ROS_ERROR("[FM2 Gather] No valid robot set found, please check robot namespace/topic settings");
        return false;
    }

    if (resolved_weights.empty()) {
        resolved_weights.assign(resolved_num, 1.0);
    } else if (resolved_weights.size() < static_cast<std::size_t>(resolved_num)) {
        resolved_weights.resize(resolved_num, 1.0);
        ROS_WARN("[FM2 Gather] robot_weights shorter than robot count, padded with 1.0");
    } else if (resolved_weights.size() > static_cast<std::size_t>(resolved_num)) {
        resolved_weights.resize(resolved_num);
        ROS_WARN("[FM2 Gather] robot_weights longer than robot count, truncated");
    }

    nh.setParam("/num_robots", resolved_num);
    nh.setParam("/robot_ids", resolved_ids);
    nh.setParam("/robot_weights", resolved_weights);
    nh.setParam("num_robots", resolved_num);
    nh.setParam("robot_ids", resolved_ids);
    nh.setParam("robot_weights", resolved_weights);
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

double getMaxVelocity(nDGridMap<FMCell, 2> grid)
   {
      double max=0;
      for(unsigned int j=0;j<grid.size();j++)
      {
         if ( grid.getCell(j).getOccupancy() )
         {
             if(grid.getCell(j).getVelocity() > max)
             {
                 max=grid.getCell(j).getVelocity();
             }
         }
      }
      return max;
   }

void plotVelocities(nDGridMap<FMCell,2> & grid, const bool flipY = true) {
            std::array<int,2> d = grid.getDimSizes();
            double max_val = getMaxVelocity(grid);
            if (!std::isfinite(max_val) || max_val <= 1e-9) {
                max_val = 1.0;
            }
            cimg_library::CImg<double> img(d[0],d[1],1,1,0);

            if (flipY) 
                // Filling the image flipping Y dim. We want now top left to be the (0,0).
                cimg_forXY(img,x,y) { img(x,y) = grid[img.width()*(img.height()-y-1)+x].getVelocity()/max_val*255; }
            else 
                cimg_forXY(img,x,y) { img(x,y) = grid[img.width()*y+x].getVelocity()/max_val*255; }	
            
            img.map( cimg_library::CImg<float>::jet_LUT256() );
            img.save_png(makeDebugFilepath("velocity.png").c_str());
            img.display("Velocity values", false);	

        }

void getDominantVelocity(nDGridMap<FMCell, 2> grid, std::vector<int>& velocity_idxs)
{
    if(grid.size() == 0)
    {
        ROS_ERROR("Grid is empty, Cant get dominant velocity");
        return;
    }
    std::priority_queue<VelocityElement, std::vector<VelocityElement>, CompareElement> max_heap;
    for(unsigned int j=0;j<grid.size();j++)
    {
        if(grid.getCell(j).getOccupancy())
        {
            VelocityElement element;
            element.idx=j;
            element.val=grid.getCell(j).getVelocity();
            max_heap.push(element);
        }
    }
    int max_num=max_heap.size();
    int k = static_cast<int>(std::ceil(max_num * dominantrate)); //取出1% 
    for(int i=0;i<k && !max_heap.empty();i++)
    {
        velocity_idxs.push_back(max_heap.top().idx);
        max_heap.pop();
    }
}

bool get_position(int num_robots, std::vector<geometry_msgs::PoseStamped>& robot_poses_, tf::TransformListener& pose_listener)
{
    //获取各个机器人在地图坐标系中的位置
   
    for (int i = 1; i <= num_robots; i++)
    {
        // 等待机器人发布tf变换 listener.lookupTransform("map", "base_link", ros::Time(0), transform)
        tf::StampedTransform transform;
        // std::string frame_id = "robot"+std::to_string(robot_ids[i-1])+ "/base_link";
        std::string frame_id = "robot"+std::to_string(robot_ids[i-1])+ "/base_footprint";
        try{
            pose_listener.waitForTransform("map", frame_id, ros::Time(0), ros::Duration(5.0));
            pose_listener.lookupTransform("map", frame_id, ros::Time(0), transform);
        }
        catch (tf::TransformException& ex)
        {
            ROS_ERROR("%s", ex.what());
            return false;
        }
        // 将机器人位置转换为地图坐标系
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = "map";
        pose_stamped.header.stamp = ros::Time::now();
        pose_stamped.pose.position.x = transform.getOrigin().x();
        pose_stamped.pose.position.y = transform.getOrigin().y();
        pose_stamped.pose.position.z = transform.getOrigin().z();
        pose_stamped.pose.orientation.x = transform.getRotation().x();
        pose_stamped.pose.orientation.y = transform.getRotation().y();
        pose_stamped.pose.orientation.z = transform.getRotation().z();
        pose_stamped.pose.orientation.w = transform.getRotation().w();
        if(robot_poses_.size() == num_robots)
            robot_poses_[i-1]=pose_stamped;
        else 
            robot_poses_.push_back(pose_stamped);
    }


   //返回true表示成功，false表示失败
   return true;
}

void update_robot_speed_estimates(const std::vector<geometry_msgs::PoseStamped>& robot_poses_)
{
    const int robot_count = static_cast<int>(robot_poses_.size());
    if (robot_count <= 0) {
        return;
    }

    if (static_cast<int>(robot_prev_positions.size()) != robot_count) {
        robot_prev_positions.assign(robot_count, std::array<double, 2>{0.0, 0.0});
        robot_prev_pose_stamp.assign(robot_count, ros::Time(0));
        robot_speed_estimates.assign(robot_count, 0.0);
    }

    const ros::Time now = ros::Time::now();
    const double alpha = std::max(0.0, std::min(1.0, peer_robot_speed_filter_alpha));
    for (int i = 0; i < robot_count; ++i) {
        const double x = robot_poses_[i].pose.position.x;
        const double y = robot_poses_[i].pose.position.y;
        const ros::Time& last_stamp = robot_prev_pose_stamp[i];
        if (last_stamp.toSec() > 0.0) {
            const double dt = (now - last_stamp).toSec();
            if (dt > 1e-3) {
                const double dx = x - robot_prev_positions[i][0];
                const double dy = y - robot_prev_positions[i][1];
                const double raw_speed = std::hypot(dx, dy) / dt;
                robot_speed_estimates[i] =
                    (1.0 - alpha) * robot_speed_estimates[i] + alpha * raw_speed;
            }
        }
        robot_prev_positions[i][0] = x;
        robot_prev_positions[i][1] = y;
        robot_prev_pose_stamp[i] = now;
    }
}

void combined_map_callback(const fm2_gather::map::ConstPtr &msg)
{
    occ.occupancyGrid = msg -> occupancyGrid;
    if(!Map_init)
    {   occ.gridSize = msg -> gridSize;
        resolution = msg -> resolution;
        occ.origin = msg -> origin;
        ndims = msg -> ndims;
        size_x_ = occ.gridSize[0];
        size_y_ = occ.gridSize[1];
        size_n_ = size_x_ * size_y_;
        dimsize[0] = occ.gridSize[0];
        dimsize[1] = occ.gridSize[1];}
    // ROS_INFO("Got combined map, resolution: %f, ndims: %d, grid size: (%d, %d)", resolution, ndims, dimsize[0], dimsize[1]);
    Map_init = true;
}

void costmap_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    occ.occupancyGrid.resize(msg->data.size());
    for(int i = 0; i < msg->data.size(); i++)
    {
        occ.occupancyGrid[i] = (msg->data[i] != 100);
    }
    if(!Map_init)
    {   occ.gridSize.resize(2);
        occ.gridSize[0] = msg -> info.width;
        occ.gridSize[1] = msg -> info.height;
        resolution = msg -> info.resolution;
        occ.origin = msg -> info.origin;
        ndims = 2;
        size_x_ = occ.gridSize[0];
        size_y_ = occ.gridSize[1];
        size_n_ = size_x_ * size_y_;
        dimsize[0] = occ.gridSize[0];
        dimsize[1] = occ.gridSize[1];}
    // ROS_INFO("Got costmap, resolution: %f, ndims: %d, grid size: (%d, %d)", resolution, ndims, dimsize[0], dimsize[1]);
    Map_init = true;
}

void dynamic_obstacle_costmap_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg, int robot_idx)
{
    if (robot_idx < 0 || robot_idx >= static_cast<int>(dynamic_obstacle_maps.size())) {
        return;
    }
    dynamic_obstacle_maps[robot_idx] = *msg;
    dynamic_obstacle_ready[robot_idx] = true;
    dynamic_obstacle_stamp[robot_idx] = msg->header.stamp;
}

bool is_dynamic_obstacle_value(const int value)
{
    if (value < 0) {
        return dynamic_obstacle_unknown_is_obstacle;
    }
    return value >= dynamic_obstacle_threshold;
}

long long make_uncertainty_kernel_key(const int hard_cells, const int soft_cells)
{
    return (static_cast<long long>(hard_cells) << 32) |
           static_cast<unsigned int>(soft_cells);
}

const std::vector<UncertaintyKernelCell>& get_uncertainty_kernel(int hard_cells, int soft_cells)
{
    hard_cells = std::max(0, hard_cells);
    soft_cells = std::max(hard_cells, soft_cells);
    if (uncertainty_kernel_resolution != static_cast<double>(resolution)) {
        uncertainty_kernel_cache.clear();
        uncertainty_kernel_resolution = static_cast<double>(resolution);
    }

    const long long key = make_uncertainty_kernel_key(hard_cells, soft_cells);
    auto it = uncertainty_kernel_cache.find(key);
    if (it != uncertainty_kernel_cache.end()) {
        return it->second;
    }

    std::vector<UncertaintyKernelCell> kernel;
    kernel.reserve((2 * soft_cells + 1) * (2 * soft_cells + 1));
    const double hard = static_cast<double>(hard_cells);
    const double soft = static_cast<double>(soft_cells);
    for (int dy = -soft_cells; dy <= soft_cells; ++dy) {
        for (int dx = -soft_cells; dx <= soft_cells; ++dx) {
            const int dist2 = dx * dx + dy * dy;
            if (dist2 > soft_cells * soft_cells) {
                continue;
            }
            double ratio = 1.0;
            if (dist2 <= hard_cells * hard_cells) {
                ratio = 0.0;
            } else if (soft_cells > hard_cells) {
                const double dist = std::sqrt(static_cast<double>(dist2));
                ratio = (dist - hard) / std::max(1e-6, soft - hard);
                ratio = std::max(0.0, std::min(1.0, ratio));
            } else {
                ratio = 0.0;
            }
            kernel.push_back(UncertaintyKernelCell{dx, dy, ratio});
        }
    }

    auto inserted = uncertainty_kernel_cache.emplace(key, std::move(kernel));
    return inserted.first->second;
}

double get_velocity_reference(const std::vector<double>& velocity_map)
{
    if (velocity_map.empty()) {
        return std::max(1e-3, v_max);
    }
    const auto it = std::max_element(velocity_map.begin(), velocity_map.end());
    const double vmax_ref = (it == velocity_map.end()) ? 0.0 : *it;
    return (vmax_ref > 1e-6) ? vmax_ref : std::max(1e-3, v_max);
}

void publish_velocity_map_msg(
    const std::vector<double>& velocity_map,
    const OccupancyMask& occupancy_grid,
    ros::Publisher& publisher)
{
    if (!publish_velocity_maps || publisher.getTopic().empty()) {
        return;
    }
    if (!Map_init || occ.gridSize.size() < 2 || resolution <= 0.0f) {
        return;
    }
    if (velocity_map.size() != static_cast<std::size_t>(size_n_)) {
        return;
    }
    if (occupancy_grid.size() != static_cast<std::size_t>(size_n_)) {
        return;
    }

    nav_msgs::OccupancyGrid msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    msg.info.resolution = resolution;
    msg.info.width = static_cast<uint32_t>(occ.gridSize[0]);
    msg.info.height = static_cast<uint32_t>(occ.gridSize[1]);
    msg.info.origin = occ.origin;
    msg.data.resize(size_n_, 0);

    const double vmax_ref = get_velocity_reference(velocity_map);
    for (int i = 0; i < size_n_; ++i) {
        if (!occupancy_grid[i]) {
            msg.data[i] = 100;
            continue;
        }
        const double vel = std::max(0.0, velocity_map[i]);
        const double risk = 1.0 - std::min(1.0, vel / std::max(1e-6, vmax_ref));
        msg.data[i] = static_cast<int8_t>(std::round(risk * 100.0));
    }
    publisher.publish(msg);
}

void publish_arrival_time_map_msg(
    nDGridMap<FMCell,2>& grid,
    ros::Publisher& publisher)
{
    if (!publish_arrival_time_maps || publisher.getTopic().empty()) {
        return;
    }
    if (!Map_init || occ.gridSize.size() < 2 || resolution <= 0.0f) {
        return;
    }
    if (grid.size() != size_n_) {
        return;
    }

    double max_value = 0.0;
    for (int i = 0; i < size_n_; ++i) {
        if (!grid.getCell(i).getOccupancy()) {
            continue;
        }
        const double val = grid.getCell(i).getValue();
        if (std::isfinite(val) && val > max_value) {
            max_value = val;
        }
    }
    if (max_value <= 1e-9) {
        max_value = 1.0;
    }

    nav_msgs::OccupancyGrid msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    msg.info.resolution = resolution;
    msg.info.width = static_cast<uint32_t>(occ.gridSize[0]);
    msg.info.height = static_cast<uint32_t>(occ.gridSize[1]);
    msg.info.origin = occ.origin;
    msg.data.resize(size_n_, 0);

    for (int i = 0; i < size_n_; ++i) {
        if (!grid.getCell(i).getOccupancy()) {
            msg.data[i] = 100;
            continue;
        }
        const double val = grid.getCell(i).getValue();
        if (!std::isfinite(val)) {
            msg.data[i] = 100;
            continue;
        }
        const double ratio = std::min(1.0, std::max(0.0, val / max_value));
        msg.data[i] = static_cast<int8_t>(std::round(ratio * 100.0));
    }
    publisher.publish(msg);
}

int apply_uncertainty_kernel_at(
    const int mx,
    const int my,
    const std::vector<UncertaintyKernelCell>& kernel,
    const double vmax_ref,
    std::vector<double>& velocity_map)
{
    int changed = 0;
    for (const auto& cell : kernel) {
        const int nx = mx + cell.dx;
        const int ny = my + cell.dy;
        if (nx < 0 || ny < 0 || nx >= size_x_ || ny >= size_y_) {
            continue;
        }
        const int nidx = ny * size_x_ + nx;
        const double cap = vmax_ref * cell.ratio;
        if (cap < velocity_map[nidx]) {
            velocity_map[nidx] = cap;
            ++changed;
        }
    }
    return changed;
}

void signal_callback(const std_msgs::UInt8::ConstPtr &msg)
{
    if(msg->data == 2)
    {
        start_compute=true;
        return;
    }
    if((msg->data - 10) > 0) // 路径重规划信号，将会给对应机器人发送一个随机扰动目标位置
    {
        replan_flag = true;
        int robot_replan_id = msg->data - 10;
        if (robot_replan_id <= 0 || robot_replan_id > static_cast<int>(robot_replan_flags.size())) {
            ROS_WARN("Invalid replan id: %d, valid range is [1, %zu]",
                     robot_replan_id, robot_replan_flags.size());
            return;
        }
        robot_replan_flags[robot_replan_id-1] = true;
        return;
    }


    else return;
}

void external_center_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    external_center_goal[0] = static_cast<float>(msg->pose.position.x);
    external_center_goal[1] = static_cast<float>(msg->pose.position.y);
    external_center_pending = true;
    external_center_dispatch_after = ros::Time(0);
    start_compute = false;
    replan_flag = false;
    std::fill(robot_replan_flags.begin(), robot_replan_flags.end(), false);
    ROS_INFO("[FM2 Gather] Received external center goal (%.3f, %.3f) from %s",
             external_center_goal[0],
             external_center_goal[1],
             external_center_goal_topic.c_str());
}


bool world2m(float wx, float wy, int& mx, int& my, float resolution_)
{
    if(wx < occ.origin.position.x || wy < occ.origin.position.y)
    {
        ROS_ERROR("World coordinates (%f, %f) are out of bounds", wx, wy); 
        return false;
    }
    mx = (int)round((wx - occ.origin.position.x) / resolution_);
    my = (int)round((wy - occ.origin.position.y) / resolution_);
    if(occ.gridSize.empty())
    {
        ROS_ERROR("Map size is not set");
        return false;
    }
    if(mx < occ.gridSize[0] && my < occ.gridSize[1])
    {
        return true;
    }
    return false;
}

bool m2world(int mx, int my, float& wx, float& wy, float resolution_)
{
    if(mx < 0 || my < 0)
    {
        ROS_ERROR("Map coordinates (%d, %d) are out of bounds", mx, my);
        return false;
    }    
    wx = occ.origin.position.x + (mx+0.5) * resolution_;
    wy = occ.origin.position.y + (my+0.5) * resolution_;
    return true;
}

bool world2m_no_log(const double wx, const double wy, int& mx, int& my)
{
    if (resolution <= 0.0f || occ.gridSize.size() < 2) {
        return false;
    }
    if (wx < occ.origin.position.x || wy < occ.origin.position.y) {
        return false;
    }
    mx = static_cast<int>(std::round((wx - occ.origin.position.x) / resolution));
    my = static_cast<int>(std::round((wy - occ.origin.position.y) / resolution));
    if (mx < 0 || my < 0 || mx >= occ.gridSize[0] || my >= occ.gridSize[1]) {
        return false;
    }
    return true;
}

bool apply_peer_robot_uncertainty_layer_to_velocity_map(
    const int robot_idx,
    const std::vector<geometry_msgs::PoseStamped>& robot_poses_,
    std::vector<double>& velocity_map)
{
    if (!use_peer_robot_uncertainty_layer) {
        return false;
    }
    if (robot_idx < 0 || robot_idx >= static_cast<int>(robot_poses_.size())) {
        return false;
    }
    if (velocity_map.size() != static_cast<std::size_t>(size_n_)) {
        return false;
    }

    const double vmax_ref = get_velocity_reference(velocity_map);
    const double hard_radius = std::max(0.0, peer_robot_hard_radius);
    int affected_peers = 0;
    int changed_cells = 0;
    for (int i = 0; i < static_cast<int>(robot_poses_.size()); ++i) {
        if (i == robot_idx) {
            continue;
        }
        int mx = 0;
        int my = 0;
        const double wx = robot_poses_[i].pose.position.x;
        const double wy = robot_poses_[i].pose.position.y;
        if (!world2m_no_log(wx, wy, mx, my)) {
            continue;
        }

        const double speed = (i < static_cast<int>(robot_speed_estimates.size())) ?
            robot_speed_estimates[i] : 0.0;
        double soft_radius = peer_robot_uncertainty_radius +
            peer_robot_speed_radius_gain * std::max(0.0, speed);
        soft_radius = std::min(peer_robot_uncertainty_max_radius, soft_radius);
        soft_radius = std::max(hard_radius, soft_radius);

        const int hard_cells = std::max(
            0, static_cast<int>(std::ceil(hard_radius / std::max(1e-6, static_cast<double>(resolution)))));
        const int soft_cells = std::max(
            hard_cells,
            static_cast<int>(std::ceil(soft_radius / std::max(1e-6, static_cast<double>(resolution)))));
        const auto& kernel = get_uncertainty_kernel(hard_cells, soft_cells);
        changed_cells += apply_uncertainty_kernel_at(mx, my, kernel, vmax_ref, velocity_map);
        ++affected_peers;
    }

    if (debug_on && affected_peers > 0) {
        ROS_INFO_THROTTLE(
            1.0,
            "[FM2 Gather] robot%d peer uncertainty applied peers=%d changed_cells=%d",
            (robot_idx < static_cast<int>(robot_ids.size()) ? robot_ids[robot_idx] : robot_idx + 1),
            affected_peers,
            changed_cells);
    }
    return changed_cells > 0;
}

void clear_robot_start_regions(
    OccupancyMask& occupancy_grid,
    const std::vector<int>& init_grid_idxs,
    const int clear_radius_cells)
{
    if (occupancy_grid.size() != static_cast<std::size_t>(size_n_)) {
        return;
    }

    const int rs = std::max(0, clear_radius_cells);
    for (const int idx : init_grid_idxs) {
        if (idx < 0 || idx >= size_n_) {
            continue;
        }
        const int cx = idx % size_x_;
        const int cy = idx / size_x_;
        for (int dy = -rs; dy <= rs; ++dy) {
            const int ny = cy + dy;
            if (ny < 0 || ny >= size_y_) {
                continue;
            }
            for (int dx = -rs; dx <= rs; ++dx) {
                const int nx = cx + dx;
                if (nx < 0 || nx >= size_x_) {
                    continue;
                }
                occupancy_grid[ny * size_x_ + nx] = true;
            }
        }
    }
}

bool build_dynamic_obstacle_planning_occupancy(
    const std::vector<geometry_msgs::PoseStamped>& robot_poses_,
    tf::TransformListener& tf_listener,
    OccupancyMask& planning_occupancy)
{
    planning_occupancy = occ.occupancyGrid;
    if (!use_dynamic_obstacle_planning_grid) {
        return true;
    }
    if (planning_occupancy.size() != static_cast<std::size_t>(size_n_)) {
        return false;
    }

    const ros::Time now = ros::Time::now();
    int used_layers = 0;
    int sampled_obstacles = 0;
    int fused_cells = 0;

    for (int robot_idx = 0; robot_idx < static_cast<int>(dynamic_obstacle_maps.size()); ++robot_idx) {
        if (!dynamic_obstacle_ready[robot_idx]) {
            continue;
        }

        if (dynamic_obstacle_timeout > 0.0 && dynamic_obstacle_stamp[robot_idx].toSec() > 0.0) {
            const double age = (now - dynamic_obstacle_stamp[robot_idx]).toSec();
            if (age > dynamic_obstacle_timeout) {
                continue;
            }
        }

        const nav_msgs::OccupancyGrid& msg = dynamic_obstacle_maps[robot_idx];
        const int width = static_cast<int>(msg.info.width);
        const int height = static_cast<int>(msg.info.height);
        const double map_res = static_cast<double>(msg.info.resolution);
        if (width <= 0 || height <= 0 || map_res <= 0.0) {
            continue;
        }
        const int total = width * height;
        if (static_cast<int>(msg.data.size()) < total) {
            continue;
        }

        const std::string frame_id = msg.header.frame_id;
        const bool need_tf = !frame_id.empty() && frame_id != "map" && frame_id != "/map";
        tf::StampedTransform tf_map_from_costmap;
        if (need_tf) {
            try {
                tf_listener.waitForTransform("map", frame_id, ros::Time(0), ros::Duration(0.1));
                tf_listener.lookupTransform("map", frame_id, ros::Time(0), tf_map_from_costmap);
            } catch (tf::TransformException& ex) {
                ROS_WARN_THROTTLE(
                    1.0,
                    "[FM2 Gather] planning-grid transform failed for robot%d frame=%s: %s",
                    (robot_idx < static_cast<int>(robot_ids.size()) ? robot_ids[robot_idx] : robot_idx + 1),
                    frame_id.c_str(),
                    ex.what());
                continue;
            }
        }

        const double ox = static_cast<double>(msg.info.origin.position.x);
        const double oy = static_cast<double>(msg.info.origin.position.y);
        const double qx = static_cast<double>(msg.info.origin.orientation.x);
        const double qy = static_cast<double>(msg.info.origin.orientation.y);
        const double qz = static_cast<double>(msg.info.origin.orientation.z);
        const double qw = static_cast<double>(msg.info.origin.orientation.w);
        const double origin_yaw = std::atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz));
        const double cos_origin = std::cos(origin_yaw);
        const double sin_origin = std::sin(origin_yaw);
        const int stride = std::max(1, dynamic_obstacle_stride);

        std::vector<int> occupied_indices;
        occupied_indices.reserve(total / std::max(1, stride * stride * 4));
        for (int row = 0; row < height; row += stride) {
            const int base = row * width;
            for (int col = 0; col < width; col += stride) {
                const int idx = base + col;
                if (idx >= total) {
                    continue;
                }
                if (is_dynamic_obstacle_value(static_cast<int>(msg.data[idx]))) {
                    occupied_indices.push_back(idx);
                }
            }
        }
        if (occupied_indices.empty()) {
            continue;
        }

        ++used_layers;
        const bool ignore_swarm_robots =
            dynamic_obstacle_ignore_swarm_robots &&
            dynamic_obstacle_swarm_ignore_radius > 0.0 &&
            !robot_poses_.empty();
        const double swarm_ignore_sq =
            dynamic_obstacle_swarm_ignore_radius * dynamic_obstacle_swarm_ignore_radius;
        const bool use_self_ignore =
            dynamic_obstacle_self_ignore_radius > 0.0 &&
            robot_idx >= 0 && robot_idx < static_cast<int>(robot_poses_.size());
        const double self_x = use_self_ignore ? robot_poses_[robot_idx].pose.position.x : 0.0;
        const double self_y = use_self_ignore ? robot_poses_[robot_idx].pose.position.y : 0.0;
        const double self_ignore_sq =
            dynamic_obstacle_self_ignore_radius * dynamic_obstacle_self_ignore_radius;

        for (int k = 0; k < static_cast<int>(occupied_indices.size()); ++k) {
            const int idx = occupied_indices[k];
            const int row = idx / width;
            const int col = idx % width;

            const double lx = (static_cast<double>(col) + 0.5) * map_res;
            const double ly = (static_cast<double>(row) + 0.5) * map_res;
            double wx = ox + cos_origin * lx - sin_origin * ly;
            double wy = oy + sin_origin * lx + cos_origin * ly;
            if (need_tf) {
                tf::Vector3 local_point(wx, wy, 0.0);
                const tf::Vector3 world_point = tf_map_from_costmap * local_point;
                wx = world_point.x();
                wy = world_point.y();
            }

            if (ignore_swarm_robots) {
                bool near_swarm_robot = false;
                for (const auto& robot_pose : robot_poses_) {
                    const double dx = wx - robot_pose.pose.position.x;
                    const double dy = wy - robot_pose.pose.position.y;
                    if (dx * dx + dy * dy <= swarm_ignore_sq) {
                        near_swarm_robot = true;
                        break;
                    }
                }
                if (near_swarm_robot) {
                    continue;
                }
            }
            if (use_self_ignore) {
                const double dx = wx - self_x;
                const double dy = wy - self_y;
                if (dx * dx + dy * dy <= self_ignore_sq) {
                    continue;
                }
            }

            int mx = 0;
            int my = 0;
            if (!world2m_no_log(wx, wy, mx, my)) {
                continue;
            }
            const int map_idx = my * size_x_ + mx;
            if (planning_occupancy[map_idx]) {
                planning_occupancy[map_idx] = false;
                ++fused_cells;
            }
            ++sampled_obstacles;
        }
    }

    if (debug_on && used_layers > 0) {
        ROS_INFO_THROTTLE(
            1.0,
            "[FM2 Gather] planning occupancy fused layers=%d sampled=%d fused_cells=%d",
            used_layers,
            sampled_obstacles,
            fused_cells);
    }
    return true;
}

bool apply_dynamic_obstacle_layer_to_velocity_map(
    const int robot_idx,
    const std::vector<geometry_msgs::PoseStamped>& robot_poses_,
    std::vector<double>& velocity_map,
    tf::TransformListener& tf_listener)
{
    if (!use_dynamic_obstacle_velocity_layer) {
        return false;
    }
    if (robot_idx < 0 || robot_idx >= static_cast<int>(dynamic_obstacle_maps.size())) {
        return false;
    }
    if (!dynamic_obstacle_ready[robot_idx]) {
        return false;
    }
    if (velocity_map.size() != static_cast<std::size_t>(size_n_)) {
        return false;
    }

    const ros::Time now = ros::Time::now();
    if (dynamic_obstacle_timeout > 0.0 && dynamic_obstacle_stamp[robot_idx].toSec() > 0.0) {
        const double age = (now - dynamic_obstacle_stamp[robot_idx]).toSec();
        if (age > dynamic_obstacle_timeout) {
            ROS_WARN_THROTTLE(
                1.0,
                "[FM2 Gather] robot%d dynamic obstacle layer stale age=%.2fs (timeout=%.2fs), skip",
                (robot_idx < static_cast<int>(robot_ids.size()) ? robot_ids[robot_idx] : robot_idx + 1),
                age,
                dynamic_obstacle_timeout);
            return false;
        }
    }

    const nav_msgs::OccupancyGrid& msg = dynamic_obstacle_maps[robot_idx];
    const int width = static_cast<int>(msg.info.width);
    const int height = static_cast<int>(msg.info.height);
    const double map_res = static_cast<double>(msg.info.resolution);
    if (width <= 0 || height <= 0 || map_res <= 0.0) {
        return false;
    }
    const int total = width * height;
    if (static_cast<int>(msg.data.size()) < total) {
        return false;
    }

    const std::string frame_id = msg.header.frame_id;
    const bool need_tf = !frame_id.empty() && frame_id != "map" && frame_id != "/map";
    tf::StampedTransform tf_map_from_costmap;
    if (need_tf) {
        try {
            tf_listener.waitForTransform("map", frame_id, ros::Time(0), ros::Duration(0.1));
            tf_listener.lookupTransform("map", frame_id, ros::Time(0), tf_map_from_costmap);
        } catch (tf::TransformException& ex) {
            ROS_WARN_THROTTLE(
                1.0,
                "[FM2 Gather] dynamic obstacle transform failed for robot%d frame=%s: %s",
                (robot_idx < static_cast<int>(robot_ids.size()) ? robot_ids[robot_idx] : robot_idx + 1),
                frame_id.c_str(),
                ex.what());
            return false;
        }
    }

    const double ox = static_cast<double>(msg.info.origin.position.x);
    const double oy = static_cast<double>(msg.info.origin.position.y);
    const double qx = static_cast<double>(msg.info.origin.orientation.x);
    const double qy = static_cast<double>(msg.info.origin.orientation.y);
    const double qz = static_cast<double>(msg.info.origin.orientation.z);
    const double qw = static_cast<double>(msg.info.origin.orientation.w);
    const double origin_yaw = std::atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz));
    const double cos_origin = std::cos(origin_yaw);
    const double sin_origin = std::sin(origin_yaw);
    const int stride = std::max(1, dynamic_obstacle_stride);

    const double hard_radius = std::max(0.0, dynamic_obstacle_inflation_radius);
    const double soft_radius = std::max(hard_radius, dynamic_obstacle_uncertainty_radius);
    const int hard_cells = std::max(
        0,
        static_cast<int>(std::ceil(hard_radius / std::max(1e-6, static_cast<double>(resolution)))));
    const int soft_cells = std::max(
        hard_cells,
        static_cast<int>(std::ceil(soft_radius / std::max(1e-6, static_cast<double>(resolution)))));
    const auto& kernel = get_uncertainty_kernel(hard_cells, soft_cells);
    const double vmax_ref = get_velocity_reference(velocity_map);

    std::vector<int> occupied_indices;
    occupied_indices.reserve(total / std::max(1, stride * stride * 4));
    for (int row = 0; row < height; row += stride) {
        const int base = row * width;
        for (int col = 0; col < width; col += stride) {
            const int idx = base + col;
            if (idx >= total) {
                continue;
            }
            if (is_dynamic_obstacle_value(static_cast<int>(msg.data[idx]))) {
                occupied_indices.push_back(idx);
            }
        }
    }
    if (occupied_indices.empty()) {
        return false;
    }

    const int max_samples = std::max(1, dynamic_obstacle_max_samples);
    const int sample_step = std::max(
        1,
        static_cast<int>(std::ceil(static_cast<double>(occupied_indices.size()) /
                                   static_cast<double>(max_samples))));
    const bool ignore_swarm_robots =
        dynamic_obstacle_ignore_swarm_robots &&
        dynamic_obstacle_swarm_ignore_radius > 0.0 &&
        !robot_poses_.empty();
    const double swarm_ignore_sq = dynamic_obstacle_swarm_ignore_radius * dynamic_obstacle_swarm_ignore_radius;
    const bool use_self_ignore =
        dynamic_obstacle_self_ignore_radius > 0.0 &&
        robot_idx >= 0 && robot_idx < static_cast<int>(robot_poses_.size());
    const double self_x = use_self_ignore ? robot_poses_[robot_idx].pose.position.x : 0.0;
    const double self_y = use_self_ignore ? robot_poses_[robot_idx].pose.position.y : 0.0;
    const double self_ignore_sq = dynamic_obstacle_self_ignore_radius * dynamic_obstacle_self_ignore_radius;

    int sampled_obstacles = 0;
    int changed_cells = 0;
    for (int k = 0; k < static_cast<int>(occupied_indices.size()); k += sample_step) {
        const int idx = occupied_indices[k];
        const int row = idx / width;
        const int col = idx % width;

        const double lx = (static_cast<double>(col) + 0.5) * map_res;
        const double ly = (static_cast<double>(row) + 0.5) * map_res;
        double wx = ox + cos_origin * lx - sin_origin * ly;
        double wy = oy + sin_origin * lx + cos_origin * ly;
        if (need_tf) {
            tf::Vector3 local_point(wx, wy, 0.0);
            const tf::Vector3 world_point = tf_map_from_costmap * local_point;
            wx = world_point.x();
            wy = world_point.y();
        }
        if (ignore_swarm_robots) {
            bool near_swarm_robot = false;
            for (const auto& robot_pose : robot_poses_) {
                const double dx = wx - robot_pose.pose.position.x;
                const double dy = wy - robot_pose.pose.position.y;
                if (dx * dx + dy * dy <= swarm_ignore_sq) {
                    near_swarm_robot = true;
                    break;
                }
            }
            if (near_swarm_robot) {
                continue;
            }
        }
        if (use_self_ignore) {
            const double dx = wx - self_x;
            const double dy = wy - self_y;
            if (dx * dx + dy * dy <= self_ignore_sq) {
                continue;
            }
        }

        int mx = 0;
        int my = 0;
        if (!world2m_no_log(wx, wy, mx, my)) {
            continue;
        }
        changed_cells += apply_uncertainty_kernel_at(mx, my, kernel, vmax_ref, velocity_map);
        ++sampled_obstacles;
    }

    if (debug_on) {
        ROS_INFO_THROTTLE(
            1.0,
            "[FM2 Gather] robot%d dynamic uncertainty sampled=%d/%zu changed_cells=%d step=%d",
            (robot_idx < static_cast<int>(robot_ids.size()) ? robot_ids[robot_idx] : robot_idx + 1),
            sampled_obstacles,
            occupied_indices.size(),
            changed_cells,
            sample_step);
    }
    return changed_cells > 0;
}

// bool InitGrids()
bool InitGrids(std::shared_ptr<nDGridMap<FMCell, 2>> grid, double resolution_)//初始化
{
    grid->resize(dimsize);
    grid->setLeafSize(resolution_);
    return true;
}

bool ResetGrids(std::shared_ptr<nDGridMap<FMCell, 2>> grid_)//重置
{
    for(int i = 0; i < grid_->size(); ++i) 
    {//重置障碍地图
        grid_->getCell(i).setOccupancy(true);
    }
    // ROS_INFO("FM2: Reset Grid Success!");
    return true;
}

bool SetGridsFromOccupancy(
    std::shared_ptr<nDGridMap<FMCell, 2>> grid,
    const OccupancyMask& occupancy_grid,
    std::vector<int>& fm2_sources,
    bool renew_source = false);

bool SetGrids(std::shared_ptr<nDGridMap<FMCell, 2>> grid, std::vector<int>& fm2_sources, bool renew_source = false) //在InitGrids之后调用
{
    return SetGridsFromOccupancy(grid, occ.occupancyGrid, fm2_sources, renew_source);
}

bool SetGridsFromOccupancy(
    std::shared_ptr<nDGridMap<FMCell, 2>> grid,
    const OccupancyMask& occupancy_grid,
    std::vector<int>& fm2_sources,
    bool renew_source)
{
    // 设置障碍
    if(renew_source)
        fm2_sources.clear();
    if(!ResetGrids(grid))//重置障碍网格
    {
        ROS_ERROR("Failed to reset grid");
    }
    if (occupancy_grid.size() != static_cast<std::size_t>(grid->size())) {
        ROS_ERROR("Failed to set grid occupancy, size mismatch occ=%zu grid=%zu",
                  occupancy_grid.size(),
                  static_cast<std::size_t>(grid->size()));
        return false;
    }
    for (int i = 0; i < static_cast<int>(occupancy_grid.size()); i++)
        {
            grid->getCell(i).setOccupancy(occupancy_grid[i]);
            if(renew_source)
            {
                 if (!occupancy_grid[i])  // occupancyGrid[i] == 0 表示该点是障碍物，1 表示该点是空
                    fm2_sources.push_back(i);
            }
               
        }  
    return true;
}

double dist(int idx1, int idx2)//用于计算两个点之间的距离
{
    int mx1, my1, mx2, my2;
    mx1 = idx1 % size_x_;
    my1 = idx1 / size_x_;
    mx2 = idx2 % size_x_;
    my2 = idx2 / size_x_;
    double dx = mx1 - mx2;
    double dy = my1 - my2;
    return sqrt(dx*dx + dy*dy);
}

//保证聚集点有足够空间
bool SpaceInflation(std::shared_ptr<nDGridMap<FMCell,2>> &grid,
    std::vector<int> fm2_sources_,
    double radius)
{
    if (radius <= 0.0 || resolution <= 0.0) {
        return true;
    }
    int r = static_cast<int>(radius / resolution);
    if(debug_on)
    {ROS_WARN("Space inflation radius/resolution: %f, radius: %f", radius/resolution, radius);}
    for(auto i: fm2_sources_)
    {
        for(int j = -r; j <= r; j++)
        {
            for(int k = -r; k <= r; k++)
            {
                int idx_ = i + j*size_x_ + k;
                if(idx_ < 0)
                    idx_ = 0;
                if(idx_ >= size_n_)
                    idx_ = size_n_-1;
                
                if(dist(idx_, i) <= radius/resolution)//&& std::find(fm2_sources_.begin(), fm2_sources_.end(), idx_) == fm2_sources_.end()
                {
                    // ROS_WARN("Setting idx: %d , dist: %f", idx_, dist(idx_, i));
                    grid->getCell(idx_).setOccupancy(false);
                    // fm2_sources_.push_back(idx_);
                }
            }
        }
    }
    return true;
}

bool gather(int num_robots, std::vector<geometry_msgs::PoseStamped>& robot_poses_, 
    std::vector<std::shared_ptr<Path>> &fmpaths_, std::array<float,2>& goal_centre_, 
    std::string mission, Gather& gatherer_, bool test_mode_,
    std::vector<std::array<float,2>>& goals_, tf::TransformListener& tf_listener)
{
    if (num_robots <= 0) {
        ROS_ERROR("Invalid num_robots=%d", num_robots);
        return false;
    }
    if (robot_poses_.size() < static_cast<std::size_t>(num_robots)) {
        ROS_ERROR("Robot poses size=%zu is smaller than num_robots=%d",
                  robot_poses_.size(),
                  num_robots);
        return false;
    }

    // 订阅合并地图话题combined_map
    std::vector<std::vector<int>> init_points;
    std::vector<std::array<int, 2>> init_grid_coords;
    std::vector<int> init_grid_idxs;
    std::vector<int> init(1);
    // nDGridMap<FMCell,2> grid_sum, grid_1, grid_2, grid_3, grid_4, grid_5, grid_6;
    auto grid_sum_ptr = std::make_shared<nDGridMap<FMCell,2>>();
    std::vector<std::shared_ptr<nDGridMap<FMCell,2>>> grid_ptrs;
    grid_ptrs.reserve(static_cast<std::size_t>(num_robots) + 1);
    for (int i = 0; i < num_robots; ++i) {
        grid_ptrs.push_back(std::make_shared<nDGridMap<FMCell,2>>());
    }
    grid_ptrs.push_back(grid_sum_ptr);

    double saturate_distance = (velocity_dmax > 0.0) ? velocity_dmax : -1.0;
    std::vector<int> fm2_sources;
    OccupancyMask planning_occupancy;
    int mx,my,idx;
    double wx;
    double wy;
    if(resolution == -1)
    {
        ROS_ERROR("Resolution is not set");
        return false;
    }
    // 初始化网格
    for(int i = 0; i < grid_ptrs.size(); i++)
    {
        InitGrids(grid_ptrs[i],  resolution);
    }

    if (!build_dynamic_obstacle_planning_occupancy(robot_poses_, tf_listener, planning_occupancy)) {
        ROS_WARN("Failed to build dynamic-aware planning occupancy, fallback to base map");
        planning_occupancy = occ.occupancyGrid;
    }
    if (planning_occupancy.size() != occ.occupancyGrid.size()) {
        ROS_ERROR("Planning occupancy size mismatch: planning=%zu base=%zu",
                  planning_occupancy.size(),
                  occ.occupancyGrid.size());
        return false;
    }

    init_points.clear();
    init_grid_coords.clear();
    init_grid_idxs.clear();
    init_grid_coords.reserve(num_robots);
    init_grid_idxs.reserve(num_robots);
    for (int i = 0; i < num_robots; i++)
    {
        wx = robot_poses_[i].pose.position.x;
        wy = robot_poses_[i].pose.position.y;
        if(!world2m(wx, wy, mx, my, resolution))
        {
            ROS_ERROR("Failed to convert world coordinates (%f, %f) to map coordinates", wx, wy);
            return false;
        }
        std::array<int, 2> coord={mx, my};
        grid_sum_ptr->coord2idx(coord,idx);

        init[0]=idx;
        init_points.push_back(init);
        init_grid_coords.push_back(coord);
        init_grid_idxs.push_back(idx);
    }

    // 保证机器人当前站位不会因为局部costmap中的观测残影被误标成障碍。
    clear_robot_start_regions(planning_occupancy, init_grid_idxs, 5);

    auto now_init = std::chrono::system_clock::now();
    std::time_t now_init_time = std::chrono::system_clock::to_time_t(now_init);
    std::tm now_init_tm = *std::localtime(&now_init_time);
    std::ostringstream init_oss;
    init_oss << std::put_time(&now_init_tm, "%Y%m%d_%H%M%S");
    std::string init_filepath = makeDebugFilepath("FM2_init_points_" + init_oss.str() + ".txt");
    std::ofstream init_out(init_filepath.c_str());
    if(!init_out.is_open())
    {
        ROS_WARN("Failed to open debug init points file: %s", init_filepath.c_str());
    }
    else
    {
        init_out << "# robot_id mx my idx\n";
        for(int i = 0; i < num_robots; i++)
        {
            int robot_id = (i < robot_ids.size()) ? robot_ids[i] : i + 1;
            init_out << robot_id << " " << init_grid_coords[i][0] << " " << init_grid_coords[i][1] << " " << init_grid_idxs[i] << "\n";
            if(debug_on)
            {
                ROS_INFO("FM2 init: robot %d grid (%d, %d) idx %d", robot_id, init_grid_coords[i][0], init_grid_coords[i][1], init_grid_idxs[i]);
            }
        }
    }

    //添加障碍物
    fm2_sources.clear();
    for(int i = 0 ; i< grid_ptrs.size(); i++)
    {
        if(i == 0)
        {
            SetGridsFromOccupancy(grid_ptrs[i], planning_occupancy, fm2_sources, true);
        }
        else
            SetGridsFromOccupancy(grid_ptrs[i], planning_occupancy, fm2_sources);
    }

// #################################FOR TESTING######################################
   
    if(test_mode_)
    {
        // GridPlotterCV::plotMap(*grid_ptrs[0]); // 绘制地图
        ROS_INFO("Computing FM2 for robot 1");
        FastMarching2< nDGridMap<FMCell,2> > fm2_1;//robot1的fm2对象
        fm2_1.setVelocityScale(v_max);
        fm2_1.setVelocityProfile(velocity_alpha, velocity_dmax);
        fm2_1.setRobotRadius(robot_radius);
        fm2_1.setVelocityMode(velocity_mode);
        fm2_1.setVelocitySigmoid(velocity_sigmoid_k, velocity_sigmoid_b);
        fm2_1.setEnvironment(grid_ptrs[0].get());
        fm2_1.setInitialAndGoalPoints(init_points[0], fm2_sources, -1);
        if(!(grid_ptrs[0]->getCell(init_points[0][0]).getOccupancy()))
        {
            ROS_ERROR("Initial point %d  is an obstacle", init_points[0][0]);
            return false;
        }
        if(std::find(fm2_sources.begin(), fm2_sources.end(), init_points[0][0]) != fm2_sources.end())
        {
            ROS_ERROR("Initial point %d  is in source", init_points[0][0]);
            return false;
        }
        fm2_1.computeFM2_gather(init_points[0] , saturate_distance);
        // GridPlotterCV::plotArrivalTimes(*grid_ptrs[0]); // 绘制robot1的时间场
        return true;
    }


//##################################################################  

    std::vector<std::shared_ptr<Path>> fmpaths;
    std::vector<std::shared_ptr<Path_v>> fmpaths_v;
    std::vector<double> base_velocity_map;
    for(int i = 0; i < num_robots; i++)
    {
        auto path = std::make_shared<Path>();
        auto path_v = std::make_shared<Path_v>();
        fmpaths.push_back(path);
        fmpaths_v.push_back(path_v);
    }
    std::vector<std::shared_ptr<FastMarching2<nDGridMap<FMCell,2>>>> fm2_solvers;
    for(int i = 0; i < num_robots; i++)
    {
        // ROS_INFO("Computing FM2 for robot %d", robot_ids[i]);
        SpaceInflation(grid_ptrs[i], fm2_sources, space_inflation_radius);
        auto fm2_solver = std::make_shared<FastMarching2<nDGridMap<FMCell,2>>>();
        fm2_solver->setVelocityScale(v_max);
        fm2_solver->setVelocityProfile(velocity_alpha, velocity_dmax);
        fm2_solver->setRobotRadius(robot_radius);
        fm2_solver->setVelocityMode(velocity_mode);
        fm2_solver->setVelocitySigmoid(velocity_sigmoid_k, velocity_sigmoid_b);
        fm2_solver->setEnvironment(grid_ptrs[i].get());
        fm2_solver->setInitialAndGoalPoints(init_points[i], fm2_sources, -1);

        if (base_velocity_map.empty()) {
            fm2_solver->computeFM2_gather(init_points[i], saturate_distance);
            base_velocity_map = fm2_solver->getVelocityMap();
            publish_velocity_map_msg(base_velocity_map, planning_occupancy, velocity_map_base_pub);

            if(debug_on)
            {
                if(plot_on)
                {
                    plotVelocities(*grid_ptrs.front());
                }
                auto now = std::chrono::system_clock::now();
                std::time_t now_time = std::chrono::system_clock::to_time_t(now);
                std::tm now_tm = *std::localtime(&now_time);
                std::ostringstream oss;
                oss << std::put_time(&now_tm, "%Y%m%d_%H%M%S");
                std::string filepath = makeDebugFilepath("VelocityMap_" + oss.str() + ".csv");
                GridWriter::saveVelocities(filepath.c_str(), *grid_ptrs.front());
            }
        }

        if (!base_velocity_map.empty()) {
            std::vector<double> robot_velocity_map = base_velocity_map;
            // Gather-point computation only considers dynamic obstacles, not swarm robots.
            apply_dynamic_obstacle_layer_to_velocity_map(i, robot_poses_, robot_velocity_map, tf_listener);
            if (i >= 0 && i < static_cast<int>(velocity_map_robot_pubs.size())) {
                publish_velocity_map_msg(robot_velocity_map, planning_occupancy, velocity_map_robot_pubs[i]);
            }
            fm2_solver->computeFM2_gather_v(init_points[i], robot_velocity_map, saturate_distance);
        } else {
            ROS_WARN("Base velocity map is empty, fallback to computeFM2_gather for robot %d",
                     (i < static_cast<int>(robot_ids.size()) ? robot_ids[i] : i + 1));
            fm2_solver->computeFM2_gather(init_points[i], saturate_distance);
            const std::vector<double> fallback_velocity_map = fm2_solver->getVelocityMap();
            if (i >= 0 && i < static_cast<int>(velocity_map_robot_pubs.size())) {
                publish_velocity_map_msg(fallback_velocity_map, planning_occupancy, velocity_map_robot_pubs[i]);
            }
        }
        if (i >= 0 && i < static_cast<int>(arrival_time_robot_pubs.size())) {
            publish_arrival_time_map_msg(*grid_ptrs[i], arrival_time_robot_pubs[i]);
        }
        if(debug_on)
        {
            std::string Map_name = "robot"+std::to_string(robot_ids[i])+"_ArrivalTimes";
            if(plot_on)
            {
                GridPlotter::plotArrivalTimes(*grid_ptrs[i],Map_name); // 绘制并保存robot_i的时间场
                }
            auto now = std::chrono::system_clock::now();
            std::time_t now_time = std::chrono::system_clock::to_time_t(now);
            std::tm now_tm = *std::localtime(&now_time);
            std::ostringstream oss;
            oss << std::put_time(&now_tm, "%Y%m%d_%H%M%S");
            std:: string Map_path = makeDebugFilepath(Map_name + oss.str() + ".csv");
            GridWriter::saveGridValues(Map_path.c_str(), *grid_ptrs[i]);
        }
        
        fm2_solvers.push_back(fm2_solver);
    }


    if(mission == "energy")
    {
        // ROS_INFO("Computing summed grid");
        for(int i = 0; i < grid_sum_ptr->size(); i++)
            {
                double sum_val = 0;
                for(int j = 0; j < num_robots; j++)
                {
                    sum_val += robot_weights[j] * grid_ptrs[j]->getCell(i).getValue();
                }
                grid_sum_ptr->getCell(i).setValue(sum_val/ weight_sum);
            }
        // ROS_INFO("grid_sum:");
        // GridPlotterCV::plotArrivalTimes(*grid_sum_ptr);
    }
    else if(mission == "fastest")
    {
        // ROS_INFO("Computing fastest gird");
        for(int j = 0 ;j<num_robots;j++)
        {
            for(int i = 0; i < grid_sum_ptr->size(); i++)
            {
                double val =  grid_ptrs[j]->getCell(i).getValue(); //robot_weights[j]/weight_sum *
                if(j ==0)
                {
                    grid_sum_ptr->getCell(i).setValue(val);
                }
                else
                {
                    if(val > grid_sum_ptr->getCell(i).getValue())
                    {
                        grid_sum_ptr->getCell(i).setValue(val);
                    }
                }
            }
        }
        //找时间最短的点
        // ROS_INFO("grid fast:");
        // GridPlotterCV::plotArrivalTimes(*grid_sum_ptr);
    }
    else if(mission == "space")
    {
        ROS_INFO("Computing space grid");
        std::vector<int> max_space_idxs;
        auto fm2_solver = std::make_shared<FastMarching2<nDGridMap<FMCell,2>>>();
        fm2_solver->setVelocityScale(v_max);
        fm2_solver->setVelocityProfile(velocity_alpha, velocity_dmax);
        fm2_solver->setRobotRadius(robot_radius);
        fm2_solver->setVelocityMode(velocity_mode);
        fm2_solver->setVelocitySigmoid(velocity_sigmoid_k, velocity_sigmoid_b);
        fm2_solver->setEnvironment(grid_sum_ptr.get()); //! 注意这里
        fm2_solver->setInitialAndGoalPoints(init_points[0], fm2_sources, -1);
        fm2_solver->computeFM2_velocity(); //计算不饱和的速度图，用于获取最大空间
        
        getDominantVelocity(*grid_sum_ptr, max_space_idxs);
        // GridPlotterCV::plotArrivalTimes(*grid_sum_ptr);
        for(int j = 0 ;j<num_robots;j++)
        {
            for(auto i: max_space_idxs)
            {
                double val = robot_weights[j]/weight_sum * grid_ptrs[j]->getCell(i).getValue();
                if(j ==0)
                {
                    grid_sum_ptr->getCell(i).setValue(val);
                }
                else
                {
                    if(val > grid_sum_ptr->getCell(i).getValue())
                    {
                        grid_sum_ptr->getCell(i).setValue(val);
                    }
                }
            }
        }
        if(debug_on)
        {
            if(plot_on)
            {

            plotVelocities(*grid_sum_ptr); // 绘制聚集速度场
            }
        auto now = std::chrono::system_clock::now();
        std::time_t now_time = std::chrono::system_clock::to_time_t(now);
        std::tm now_tm = *std::localtime(&now_time);
        std::ostringstream oss;
        oss << std::put_time(&now_tm, "%Y%m%d_%H%M%S");
        std::string filepath = makeDebugFilepath("SumVelocityMap_" + oss.str() + ".csv");
            GridWriter::saveVelocityValues(filepath.c_str(), *grid_sum_ptr);
        }
        
    }

    else
    {
        ROS_ERROR("Invalid mission type: %s", mission.c_str());
        return false;
    }
    // GridPlotterCV::plotMap(*grid_sum_ptr); // 绘制聚集点占据情况
    // GridPlotterCV::plotArrivalTimes(*grid_sum_ptr); // 绘制聚集信息场
    if(debug_on)
    {
        
        auto now = std::chrono::system_clock::now();
        std::time_t now_time = std::chrono::system_clock::to_time_t(now);
        std::tm now_tm = *std::localtime(&now_time);
        std::ostringstream oss;
        oss << std::put_time(&now_tm, "%Y%m%d_%H%M%S");
        std::string filepath = makeDebugFilepath("Cost_map_" + oss.str() + ".csv");
        GridWriter::saveGridValues(filepath.c_str(), *grid_sum_ptr);
        if(plot_on)
        {
        GridPlotter::plotMap(*grid_sum_ptr, "Combined_map"); // 绘制聚集点占据情况
        GridPlotter::plotArrivalTimes(*grid_sum_ptr, "Cost_map"); // 绘制聚集代价图
        }
    }
    publish_arrival_time_map_msg(*grid_sum_ptr, arrival_time_sum_pub);

    const double gather_inflation_radius =
        (gather_radius > 0.0) ? gather_radius : (gatherer_.getHypotenus() + 0.1);
    SpaceInflation(grid_sum_ptr, fm2_sources, gather_inflation_radius);
    if(debug_on)
    {
        std::string Map_name = "Inflated_map";
        if(plot_on)
        {
            GridPlotter::plotMap(*grid_sum_ptr, Map_name); // 绘制聚集点占据情况
        }
    }
    // 计算聚集中心
    int min_idx = gatherer_.getMinIdx(*grid_sum_ptr); //聚集代价最小
    Min_idx = min_idx;
    

    // for(int i = 0; i < num_robots; i++)
    // {
    //     fm2_solvers[i]->computePath(fmpaths[i].get(), fmpaths_v[i].get(), min_idx);
    // }
    
    
    // GridPlotterCV::plotMapPath(*grid_1_ptr, fmpaths); // 绘制聚集路径
    fmpaths_=fmpaths;
    std::array<int,2> c_coord;
    grid_sum_ptr->idx2coord(min_idx, c_coord); //获取聚集中心地图坐标
    C_coord = c_coord;
    float gx, gy;
    m2world(c_coord[0], c_coord[1], gx, gy, resolution);//聚集中心世界坐标
    //分别计算各个机器人目标坐标（不同队形）
    goal_centre_[0] = gx;
    goal_centre_[1] = gy;
    goals_.clear();
    if (publish_center_goal_only) {
        goals_.assign(num_robots, goal_centre_);
    }
    if(debug_on)
    {
        ROS_WARN("### Min_idx: %d", min_idx);
        ROS_INFO("Center position: (%f, %f)", gx, gy);
        ROS_INFO("Centor coord: (%d, %d)", c_coord[0], c_coord[1]);
    }

    // GridPlotterCV::plotMap(*grid_sum_ptr); // 绘制聚集点占据情况
    //聚集队形中各个机器人目标位置生成函数
    // gatherer_.GenerateGoals(robot_poses_, goal_centre_, goals_);

    if(debug_on)
    {
        std::vector<int> goals_idx;
        if (publish_center_goal_only) {
            goals_idx.assign(num_robots, min_idx);
        } else {
            gatherer_.GenerateGoals_idx(robot_poses_, goal_centre_, goals_idx);
            if(reverse && goals_idx.size() >= 2)
            {
                int temp;
                temp = goals_idx[0];
                goals_idx[0] = goals_idx[1];
                goals_idx[1] = temp;
            }
        }

        const std::size_t debug_count = std::min(
            std::min(goals_idx.size(), fm2_solvers.size()),
            std::min(
                std::min(fmpaths.size(), fmpaths_v.size()),
                std::min(grid_ptrs.size(), robot_poses_.size())));
        for(std::size_t i = 0; i < debug_count; i++)
        {
            if (goals_idx[i] < 0 || goals_idx[i] >= grid_ptrs[i]->size()) {
                const int robot_id = (i < robot_ids.size()) ? robot_ids[i] : static_cast<int>(i) + 1;
                ROS_WARN("Skip robot%d debug path due to invalid goal idx %d (grid size %d)",
                         robot_id,
                         goals_idx[i],
                         grid_ptrs[i]->size());
                continue;
            }

            fmpaths[i]->clear();
            fmpaths_v[i]->clear();
            fm2_solvers[i]->computePath(fmpaths[i].get(), fmpaths_v[i].get(), goals_idx[i]);
            const int robot_id = (i < robot_ids.size()) ? robot_ids[i] : static_cast<int>(i) + 1;
            if (fmpaths[i]->empty()) {
                ROS_WARN("Skip robot%d debug path save because path backtracking returned empty",
                         robot_id);
                continue;
            }
            if (fmpaths[i]->size() <= 2) {
                ROS_WARN("Robot%d debug path was truncated to %zu points",
                         robot_id,
                         fmpaths[i]->size());
            }
            auto now = std::chrono::system_clock::now();
            std::time_t now_time = std::chrono::system_clock::to_time_t(now);
            std::tm now_tm = *std::localtime(&now_time);
            std::ostringstream oss;
            oss << std::put_time(&now_tm, "%Y%m%d_%H%M%S");
            std::string filepath = makeDebugFilepath("robot" + std::to_string(robot_id) + "_path" + oss.str() + ".csv");
            GridWriter::savePath(filepath.c_str(), *grid_ptrs[i], *fmpaths[i]);
            if(plot_on)
            {
                try {
                    GridPlotter::plotArrivalTimesPath(*grid_ptrs[i], *fmpaths[i]);
                } catch (const cimg_library::CImgException& ex) {
                    ROS_WARN("Skip robot%d path plot due to CImg exception: %s",
                             robot_id,
                             ex.what());
                }
            }
        }
        if (debug_count < goals_idx.size()) {
            ROS_WARN("Skip %zu debug paths because robot/path/grid vector count is smaller than goal count",
                     goals_idx.size() - debug_count);
        }
    }

    //返回true表示成功，false表示失败
return true;
}   

geometry_msgs::PoseStamped make_goal_pose(const std::array<float,2>& goal_, const ros::Time& stamp)
{
    geometry_msgs::PoseStamped goal_pose;
    goal_pose.header.frame_id = "map";
    goal_pose.header.stamp = stamp;
    goal_pose.pose.position.x = goal_[0];
    goal_pose.pose.position.y = goal_[1];
    goal_pose.pose.position.z = 0.0;
    goal_pose.pose.orientation.x = 0.0;
    goal_pose.pose.orientation.y = 0.0;
    goal_pose.pose.orientation.z = 0.0;
    goal_pose.pose.orientation.w = 1.0;
    return goal_pose;
}

double resolve_goal_occupied_staging_radius(float hypotenuse)
{
    if (goal_occupied_staging_radius > 0.0) {
        return goal_occupied_staging_radius;
    }
    const double base_radius =
        (gather_radius > 0.0) ? gather_radius : (static_cast<double>(hypotenuse) + 0.1);
    return std::max(base_radius + goal_occupied_staging_margin, 0.3);
}

bool publish_goals(std::vector<std::shared_ptr<MoveBaseController>>& controllers_,
                   const std::vector<std::array<float,2>>& goals_,
                   const std::array<float,2>& center_goal_,
                   const bool use_center_only,
                   const std::set<int>& skip_robot_ids = std::set<int>())
{
    if (controllers_.size() < robot_ids.size()) {
        ROS_ERROR("Controller count (%zu) is smaller than robot_ids size (%zu)",
                  controllers_.size(), robot_ids.size());
        return false;
    }
    if (!use_center_only && goals_.size() < robot_ids.size()) {
        ROS_ERROR("Goal count (%zu) is smaller than robot_ids size (%zu)",
                  goals_.size(), robot_ids.size());
        return false;
    }

    const ros::Time stamp = ros::Time::now();
    for(int i = 0; i < robot_ids.size(); i++)
    {
        const int robot_id = robot_ids[i];
        if (skip_robot_ids.find(robot_id) != skip_robot_ids.end()) {
            ROS_INFO("[FM2 Gather] Skip goal publish for robot %d because shape assembly is active.", robot_id);
            continue;
        }
        const std::array<float, 2> target =
            use_center_only ? center_goal_ : goals_[i];

        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose = make_goal_pose(target, stamp);
        controllers_[i]->sendGoal(goal);
    }
    return true;
}

bool publish_random_goal(std::shared_ptr<MoveBaseController>& controller_, std::array<float,2> goal_)
{
    std::random_device rd;  // 获取真随机数种子
    std::mt19937 gen(rd()); // 使用Mersenne Twister算法
    std::uniform_real_distribution<> dis(-GOAL_RAND_RANGE, GOAL_RAND_RANGE);

    geometry_msgs::PoseStamped goal_pose;
    goal_pose.header.frame_id = "map";
    goal_pose.header.stamp = ros::Time::now();
    goal_pose.pose.position.x = goal_[0]+dis(gen);
    goal_pose.pose.position.y = goal_[1]+dis(gen);
    goal_pose.pose.position.z = 0;
    goal_pose.pose.orientation.x = 0;
    goal_pose.pose.orientation.y = 0;
    goal_pose.pose.orientation.z = 0;
    goal_pose.pose.orientation.w = 1;
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose = goal_pose;
    controller_->sendGoal(goal);
    return true;
}

bool publish_goal_occupied_staging_goal(std::shared_ptr<MoveBaseController>& controller_,
                                        const std::array<float,2>& center_goal_,
                                        const geometry_msgs::PoseStamped& robot_pose,
                                        int robot_index,
                                        int robot_count,
                                        double staging_radius)
{
    const double cx = static_cast<double>(center_goal_[0]);
    const double cy = static_cast<double>(center_goal_[1]);
    const double px = robot_pose.pose.position.x;
    const double py = robot_pose.pose.position.y;
    const double dx = px - cx;
    const double dy = py - cy;
    const double dist = std::sqrt(dx * dx + dy * dy);
    if (dist <= staging_radius) {
        ROS_INFO("Robot %d already inside shape-entry radius %.2f around gather center, wait for shape-assembly takeover",
                 robot_index + 1, staging_radius);
        return true;
    }

    const int safe_count = std::max(1, robot_count);
    const double kTwoPi = 6.28318530717958647692;
    const double slot_angle = (kTwoPi * static_cast<double>(robot_index)) /
                              static_cast<double>(safe_count);
    double target_angle = slot_angle;
    if (dist > 1e-6) {
        const double radial_angle = std::atan2(dy, dx);
        const double blend = 0.25;
        target_angle = std::atan2(
            (1.0 - blend) * std::sin(radial_angle) + blend * std::sin(slot_angle),
            (1.0 - blend) * std::cos(radial_angle) + blend * std::cos(slot_angle));
    }

    std::array<float, 2> staging_goal = {
        static_cast<float>(cx + staging_radius * std::cos(target_angle)),
        static_cast<float>(cy + staging_radius * std::sin(target_angle))
    };
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose = make_goal_pose(staging_goal, ros::Time::now());
    controller_->sendGoal(goal);
    ROS_WARN("Robot %d goal occupied by peers, send staging goal (%.3f, %.3f) around center (%.3f, %.3f) radius=%.2f",
             robot_index + 1,
             staging_goal[0],
             staging_goal[1],
             center_goal_[0],
             center_goal_[1],
             staging_radius);
    return true;
}
    
// bool make_plans(std::vector<std::shared_ptr<MoveBaseController>>& controllers_, geometry_msgs::PoseStamped robot_pose, std::vector<std::array<float,2>> goals_, std::vector<nav_msgs::Path>& paths)
// {
//     for(int i = 0; i < robot_ids.size(); i++)
// }


bool controllers_check(std::vector<std::shared_ptr<MoveBaseController>> &controllers_)
{
    bool all_succ = true;
    for(int i = 0; i < robot_ids.size(); i++)
    {

        if(controllers_[i]->getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            all_succ = false;
        }
    }
    if(all_succ)
    {
        ROS_INFO("All robots are in succeeded state");
        return false; //全部完成，发送聚集停止
    }
    return true;
}




int main(int argc, char** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "gather_node");
    ros::NodeHandle nh;

    if (!resolveRobotConfig(nh, num_robots, robot_ids, robot_weights)) {
        return 1;
    }
    std::ostringstream robot_ids_stream;
    for (std::size_t i = 0; i < robot_ids.size(); ++i) {
        if (i > 0) {
            robot_ids_stream << ",";
        }
        robot_ids_stream << robot_ids[i];
    }
    ROS_INFO("[FM2 Gather] Resolved robots: num_robots=%d ids=[%s]",
             num_robots, robot_ids_stream.str().c_str());

    nh.param("dominantrate", dominantrate, dominantrate);
    nh.param("v_max", v_max, v_max);
    nh.param("velocity_alpha", velocity_alpha, velocity_alpha);
    nh.param("velocity_dmax", velocity_dmax, velocity_dmax);
    nh.param("robot_radius", robot_radius, robot_radius);
    nh.param("velocity_mode", velocity_mode, velocity_mode);
    nh.param("velocity_sigmoid_k", velocity_sigmoid_k, velocity_sigmoid_k);
    nh.param("velocity_sigmoid_b", velocity_sigmoid_b, velocity_sigmoid_b);
    nh.param("space_inflation_radius", space_inflation_radius, space_inflation_radius);
    nh.param("gather_radius", gather_radius, gather_radius);
    nh.param("goal_occupied_staging_radius",
             goal_occupied_staging_radius,
             goal_occupied_staging_radius);
    nh.param("goal_occupied_staging_margin",
             goal_occupied_staging_margin,
             goal_occupied_staging_margin);
    nh.param("publish_center_goal_only", publish_center_goal_only, publish_center_goal_only);
    nh.param("gather_center_topic", gather_center_topic, gather_center_topic);
    nh.param("use_combined_map", use_combined_map, use_combined_map);
    nh.param("combined_map_topic", combined_map_topic, combined_map_topic);
    nh.param("base_map_topic", base_map_topic, base_map_topic);
    nh.param("use_dynamic_obstacle_planning_grid",
             use_dynamic_obstacle_planning_grid,
             use_dynamic_obstacle_planning_grid);
    nh.param("use_dynamic_obstacle_velocity_layer",
             use_dynamic_obstacle_velocity_layer,
             use_dynamic_obstacle_velocity_layer);
    nh.param("dynamic_obstacle_topic_suffix",
             dynamic_obstacle_topic_suffix,
             dynamic_obstacle_topic_suffix);
    nh.param("dynamic_obstacle_threshold",
             dynamic_obstacle_threshold,
             dynamic_obstacle_threshold);
    nh.param("dynamic_obstacle_unknown_is_obstacle",
             dynamic_obstacle_unknown_is_obstacle,
             dynamic_obstacle_unknown_is_obstacle);
    nh.param("dynamic_obstacle_timeout",
             dynamic_obstacle_timeout,
             dynamic_obstacle_timeout);
    nh.param("dynamic_obstacle_inflation_radius",
             dynamic_obstacle_inflation_radius,
             dynamic_obstacle_inflation_radius);
    nh.param("dynamic_obstacle_uncertainty_radius",
             dynamic_obstacle_uncertainty_radius,
             dynamic_obstacle_uncertainty_radius);
    nh.param("dynamic_obstacle_self_ignore_radius",
             dynamic_obstacle_self_ignore_radius,
             dynamic_obstacle_self_ignore_radius);
    nh.param("dynamic_obstacle_ignore_swarm_robots",
             dynamic_obstacle_ignore_swarm_robots,
             dynamic_obstacle_ignore_swarm_robots);
    nh.param("dynamic_obstacle_swarm_ignore_radius",
             dynamic_obstacle_swarm_ignore_radius,
             dynamic_obstacle_swarm_ignore_radius);
    nh.param("dynamic_obstacle_stride",
             dynamic_obstacle_stride,
             dynamic_obstacle_stride);
    nh.param("dynamic_obstacle_max_samples",
             dynamic_obstacle_max_samples,
             dynamic_obstacle_max_samples);
    nh.param("use_peer_robot_uncertainty_layer",
             use_peer_robot_uncertainty_layer,
             use_peer_robot_uncertainty_layer);
    nh.param("peer_robot_hard_radius",
             peer_robot_hard_radius,
             peer_robot_hard_radius);
    nh.param("peer_robot_uncertainty_radius",
             peer_robot_uncertainty_radius,
             peer_robot_uncertainty_radius);
    nh.param("peer_robot_speed_radius_gain",
             peer_robot_speed_radius_gain,
             peer_robot_speed_radius_gain);
    nh.param("peer_robot_uncertainty_max_radius",
             peer_robot_uncertainty_max_radius,
             peer_robot_uncertainty_max_radius);
    nh.param("peer_robot_speed_filter_alpha",
             peer_robot_speed_filter_alpha,
             peer_robot_speed_filter_alpha);
    nh.param("publish_velocity_maps",
             publish_velocity_maps,
             publish_velocity_maps);
    nh.param("velocity_map_topic_prefix",
             velocity_map_topic_prefix,
             velocity_map_topic_prefix);
    nh.param("publish_arrival_time_maps",
             publish_arrival_time_maps,
             publish_arrival_time_maps);
    nh.param("arrival_time_topic_prefix",
             arrival_time_topic_prefix,
             arrival_time_topic_prefix);

    if (!dynamic_obstacle_topic_suffix.empty() &&
        dynamic_obstacle_topic_suffix.front() == '/') {
        dynamic_obstacle_topic_suffix.erase(0, 1);
    }
    dynamic_obstacle_stride = std::max(1, dynamic_obstacle_stride);
    dynamic_obstacle_max_samples = std::max(1, dynamic_obstacle_max_samples);
    velocity_alpha = std::max(0.0, std::min(1.0, velocity_alpha));
    velocity_dmax = std::max(0.0, velocity_dmax);
    robot_radius = std::max(0.0, robot_radius);
    velocity_mode = (velocity_mode == 1) ? 1 : 0;
    velocity_sigmoid_k = std::max(0.0, velocity_sigmoid_k);
    velocity_sigmoid_b = std::max(0.0, velocity_sigmoid_b);
    goal_occupied_staging_radius = std::max(-1.0, goal_occupied_staging_radius);
    goal_occupied_staging_margin = std::max(0.0, goal_occupied_staging_margin);
    dynamic_obstacle_threshold = std::max(0, std::min(100, dynamic_obstacle_threshold));
    dynamic_obstacle_inflation_radius = std::max(0.0, dynamic_obstacle_inflation_radius);
    dynamic_obstacle_uncertainty_radius =
        std::max(dynamic_obstacle_inflation_radius, dynamic_obstacle_uncertainty_radius);
    dynamic_obstacle_self_ignore_radius = std::max(0.0, dynamic_obstacle_self_ignore_radius);
    dynamic_obstacle_swarm_ignore_radius = std::max(0.0, dynamic_obstacle_swarm_ignore_radius);
    peer_robot_hard_radius = std::max(0.0, peer_robot_hard_radius);
    peer_robot_uncertainty_radius = std::max(peer_robot_hard_radius, peer_robot_uncertainty_radius);
    peer_robot_uncertainty_max_radius =
        std::max(peer_robot_uncertainty_radius, peer_robot_uncertainty_max_radius);
    peer_robot_speed_radius_gain = std::max(0.0, peer_robot_speed_radius_gain);
    peer_robot_speed_filter_alpha = std::max(0.0, std::min(1.0, peer_robot_speed_filter_alpha));
    if (velocity_map_topic_prefix.empty()) {
        velocity_map_topic_prefix = "/fm2_gather/velocity_map";
    }
    while (!velocity_map_topic_prefix.empty() &&
           velocity_map_topic_prefix.back() == '/') {
        velocity_map_topic_prefix.pop_back();
    }
    if (arrival_time_topic_prefix.empty()) {
        arrival_time_topic_prefix = "/fm2_gather/arrival_time";
    }
    while (!arrival_time_topic_prefix.empty() &&
           arrival_time_topic_prefix.back() == '/') {
        arrival_time_topic_prefix.pop_back();
    }
    if (use_peer_robot_uncertainty_layer) {
        ROS_WARN("[FM2 Gather] use_peer_robot_uncertainty_layer is enabled, but gather-point computation ignores swarm robots in velocity maps.");
    }
    if (use_combined_map && use_dynamic_obstacle_planning_grid) {
        ROS_WARN("[FM2 Gather] use_combined_map=true, disable temporary dynamic planning-grid overlay to avoid duplicate obstacle injection");
        use_dynamic_obstacle_planning_grid = false;
    }
    ROS_INFO("[FM2 Gather] Dynamic obstacle planning grid=%s, velocity layer=%s, ignore_swarm_robots=%s, swarm_ignore_radius=%.2f",
             use_dynamic_obstacle_planning_grid ? "true" : "false",
             use_dynamic_obstacle_velocity_layer ? "true" : "false",
             dynamic_obstacle_ignore_swarm_robots ? "true" : "false",
             dynamic_obstacle_swarm_ignore_radius);

    bool got_position = false;
    bool test_mode = false;
    float hypotenuse = 0.4; // 机器人对角线长度（圆形队列半径）
    nh.getParam("hypo", hypotenuse);
    ROS_INFO("Hypotenuse: %f", hypotenuse);
    std::string mission;
    nh.getParam("mission", mission);
    ROS_INFO("Mission: %s", mission.c_str());
    nh.param("pub_once", pub_once, false);
    nh.param("reverse", reverse, false);
    nh.param("test_mode", test_mode, false);
    nh.param("debug_on", debug_on, true);
    nh.param("publish_goal",publish_goal, true);
    nh.param("plot_on", plot_on, true);
    nh.param("save_data", save_data, false);
    nh.param("external_center_goal_topic", external_center_goal_topic, external_center_goal_topic);
    nh.param("external_center_sync_wait", external_center_sync_wait, external_center_sync_wait);
    std::vector<std::shared_ptr<Path>> fmpaths;
    std::vector<geometry_msgs::PoseStamped> robot_poses;
    std::vector<std::shared_ptr<MoveBaseController>> controllers;
    // 创建TransformListener对象，用于监听tf变换
    robot_replan_flags.assign(num_robots, false); //用于判断机器人是否需要在特殊情况下重新规划路径
    tf::TransformListener tf_listener; //    用于监听tf变换
    ros::Subscriber Costmap_sub;
    ros::Subscriber CombinedMap_sub;
    if (use_combined_map) {
        if (combined_map_topic.empty()) {
            combined_map_topic = "/combined_map";
        }
        ROS_INFO("Subscribing combined map topic: %s", combined_map_topic.c_str());
        CombinedMap_sub = nh.subscribe(combined_map_topic, 1, &combined_map_callback);
    } else {
        if (base_map_topic.empty()) {
            base_map_topic = "/map";
        }
        ROS_INFO("Subscribing base map topic: %s", base_map_topic.c_str());
        Costmap_sub = nh.subscribe(base_map_topic, 1, &costmap_callback);
    }
    ros::Subscriber sub2 = nh.subscribe("/gather_signal",1 ,&signal_callback);
    ros::Subscriber external_center_sub =
        nh.subscribe(external_center_goal_topic, 1, &external_center_callback);

    dynamic_obstacle_maps.assign(num_robots, nav_msgs::OccupancyGrid());
    dynamic_obstacle_ready.assign(num_robots, false);
    dynamic_obstacle_stamp.assign(num_robots, ros::Time(0));
    robot_prev_positions.assign(num_robots, std::array<double, 2>{0.0, 0.0});
    robot_prev_pose_stamp.assign(num_robots, ros::Time(0));
    robot_speed_estimates.assign(num_robots, 0.0);
    std::vector<ros::Subscriber> dynamic_obstacle_subs;
    if (use_dynamic_obstacle_velocity_layer || use_dynamic_obstacle_planning_grid) {
        dynamic_obstacle_subs.reserve(num_robots);
        for (int i = 0; i < num_robots; ++i) {
            const std::string topic_name =
                makeRobotTopic(robot_ids[i], dynamic_obstacle_topic_suffix);
            ROS_INFO("Subscribing dynamic obstacle topic: %s", topic_name.c_str());
            dynamic_obstacle_subs.push_back(
                nh.subscribe<nav_msgs::OccupancyGrid>(
                    topic_name,
                    1,
                    boost::bind(dynamic_obstacle_costmap_callback, _1, i)));
        }
    }

    weight_sum = 0.0;
    for (auto i:robot_weights)
    {
        weight_sum += i;
    }
    if (weight_sum <= 0.0) {
        ROS_WARN("robot_weights sum is invalid (%.3f), fallback to uniform", weight_sum);
        robot_weights.assign(num_robots, 1.0);
        weight_sum = static_cast<double>(num_robots);
    }
    // MOVE_BASE控制器
    
    for (auto i: robot_ids)
    {
        std::string action_server_namespace = makeRobotNamespace(i);
        ROS_INFO("Subscribing to %s/move_base", action_server_namespace.c_str());
        auto controller = std::make_shared<MoveBaseController>(action_server_namespace);
        controllers.push_back(controller);
    }
    //聚集开始信号话题发布者
    ros::Publisher pub_start = nh.advertise<std_msgs::UInt8>("/gather_started", 1, true);
    ros::Publisher pub_center = nh.advertise<geometry_msgs::PoseStamped>(gather_center_topic, 1, true);
    std_msgs::UInt8 gather_state_msg;
    gather_state_msg.data = 0;
    pub_start.publish(gather_state_msg);
    if (publish_velocity_maps) {
        velocity_map_base_pub =
            nh.advertise<nav_msgs::OccupancyGrid>(velocity_map_topic_prefix + "/base", 1, true);
        velocity_map_robot_pubs.clear();
        velocity_map_robot_pubs.reserve(num_robots);
        for (int i = 0; i < num_robots; ++i) {
            const int robot_id = (i < static_cast<int>(robot_ids.size())) ? robot_ids[i] : (i + 1);
            const std::string topic =
                velocity_map_topic_prefix + "/robot" + std::to_string(robot_id);
            velocity_map_robot_pubs.push_back(
                nh.advertise<nav_msgs::OccupancyGrid>(topic, 1, true));
            ROS_INFO("Publishing velocity map topic: %s", topic.c_str());
        }
        ROS_INFO("Publishing velocity base topic: %s/base", velocity_map_topic_prefix.c_str());
    } else {
        velocity_map_robot_pubs.clear();
    }
    if (publish_arrival_time_maps) {
        arrival_time_sum_pub =
            nh.advertise<nav_msgs::OccupancyGrid>(arrival_time_topic_prefix + "/combined", 1, true);
        arrival_time_robot_pubs.clear();
        arrival_time_robot_pubs.reserve(num_robots);
        for (int i = 0; i < num_robots; ++i) {
            const int robot_id = (i < static_cast<int>(robot_ids.size())) ? robot_ids[i] : (i + 1);
            const std::string topic =
                arrival_time_topic_prefix + "/robot" + std::to_string(robot_id);
            arrival_time_robot_pubs.push_back(
                nh.advertise<nav_msgs::OccupancyGrid>(topic, 1, true));
            ROS_INFO("Publishing arrival-time topic: %s", topic.c_str());
        }
        ROS_INFO("Publishing arrival-time combined topic: %s/combined", arrival_time_topic_prefix.c_str());
    } else {
        arrival_time_robot_pubs.clear();
    }

    // 设置循环频率
    ros::Rate rate(3); // 3Hz
    // std::string mission = "fastest";//fastest , energy, space
    bool last_stop_path_planning = false;
    
    while (ros::ok())
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
                ROS_WARN_STREAM("[FM2 Gather] Shape takeover active, ignore new path planning. active_ids=[" << oss.str() << "]");
            } else {
                ROS_INFO("[FM2 Gather] Shape takeover cleared, resume path planning.");
            }
        }
        last_stop_path_planning = stop_path_planning;
        try
        {
            got_position = get_position(num_robots, robot_poses, tf_listener);
            if (got_position && Map_init)
            {
                if (stop_path_planning && !external_center_pending) {
                    if (start_compute || replan_flag) {
                        start_compute = false;
                        replan_flag = false;
                        std::fill(robot_replan_flags.begin(), robot_replan_flags.end(), false);
                        ROS_INFO("[FM2 Gather] Cleared pending start/replan signals due to shape takeover.");
                    }
                    rate.sleep();
                    continue;
                }
                Gather gatherer(num_robots, hypotenuse, resolution, occ.origin.position.x, occ.origin.position.y, size_x_, size_y_); //负责生成聚集目标点中的各种操作
                if (external_center_pending) {
                    if (external_center_dispatch_after.isZero()) {
                        gather_state_msg.data = 0;
                        pub_start.publish(gather_state_msg);
                        pub_center.publish(make_goal_pose(external_center_goal, ros::Time::now()));
                        external_center_dispatch_after = ros::Time::now() + ros::Duration(std::max(0.0, external_center_sync_wait));
                        ROS_INFO("[FM2 Gather] Published external center and waiting %.2fs for shape state refresh.",
                                 std::max(0.0, external_center_sync_wait));
                        rate.sleep();
                        continue;
                    }
                    if (ros::Time::now() < external_center_dispatch_after) {
                        rate.sleep();
                        continue;
                    }

                    if (publish_center_goal_only) {
                        goals.assign(num_robots, external_center_goal);
                    } else {
                        gatherer.GenerateGoals(robot_poses, external_center_goal, goals);
                    }
                    pub_center.publish(make_goal_pose(external_center_goal, ros::Time::now()));
                    if (publish_goal) {
                        publish_goals(controllers, goals, external_center_goal, publish_center_goal_only, shape_active_robot_ids);
                    }
                    gather_state_msg.data = 1;
                    pub_start.publish(gather_state_msg);
                    external_center_pending = false;
                    external_center_dispatch_after = ros::Time(0);
                    ROS_INFO("[FM2 Gather] Dispatched external center goals, active shape robots skipped=%zu.",
                             shape_active_robot_ids.size());
                    rate.sleep();
                    continue;
                }
                if(!controllers_check(controllers)) //检查控制器是否全部完成任务，/gather_start中发布0 
                {
                    std_msgs::UInt8 msg;
                    msg.data = 0;
                    pub_start.publish(msg);
                }
                if(test_mode) //测试模式
                {
                    ROS_INFO("Start test");
                    std::array<float,2> goal_centre;
                    gather(num_robots, robot_poses, fmpaths, goal_centre, mission,
                           gatherer, test_mode, goals, tf_listener);
                }

                if(start_compute) //开始计算聚集点
                {
                    std_msgs::UInt8 msg;
                    msg.data = 0;
                    pub_start.publish(msg);
                    for(int i=0;i<robot_ids.size();i++)
                    {
                        // controllers[i]->cancelGoal();
                        controllers[i]->emergencyStop();
                    }
                    ROS_INFO("Start mission: %s", mission.c_str());
                    std::array<float,2> goal_centre;
                    auto start = std::chrono::high_resolution_clock::now();
                    bool succ = gather(num_robots, robot_poses, fmpaths, goal_centre, mission,
                                       gatherer, test_mode, goals, tf_listener);
        
                    auto end = std::chrono::high_resolution_clock::now();
                    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
                    ROS_INFO("Gather Compute time: %ld ms", duration.count());
                    if(succ)
                    {
                        if (publish_center_goal_only) {
                            goals.assign(num_robots, goal_centre);
                        } else {
                            // ROS_INFO("Generate goals...");
                            // gatherer.GenerateGoals_p(fmpaths, goal_centre, goals);
                            gatherer.GenerateGoals(robot_poses, goal_centre, goals);
                            if(reverse && goals.size() >= 2)
                            {
                                std::array<float,2UL> temp = goals[0];
                                goals[0] = goals[1];
                                goals[1] = temp;
                            }
                        }
                        if (goals.size() < static_cast<std::size_t>(num_robots)) {
                            ROS_WARN("Generated goals size=%zu smaller than num_robots=%d",
                                     goals.size(), num_robots);
                        }
                        std::ostringstream goals_stream;
                        for (std::size_t i = 0; i < goals.size(); ++i) {
                            if (i > 0) {
                                goals_stream << ", ";
                            }
                            goals_stream << "(" << goals[i][0] << ", " << goals[i][1] << ")";
                        }
                        ROS_WARN("GOALS: %s", goals_stream.str().c_str());

                        const geometry_msgs::PoseStamped center_goal_pose =
                            make_goal_pose(goal_centre, ros::Time::now());
                        pub_center.publish(center_goal_pose);
                    }
                    if(publish_goal && succ)
                    {
                        publish_goals(controllers, goals, goal_centre, publish_center_goal_only, shape_active_robot_ids);
                        if(pub_once)
                        {
                            start_compute = false;
                            msg.data = 1; //发布聚集开始信号
                            pub_start.publish(msg);
                            return 0;
                         }
                    }
                    
                    start_compute = false;
                    
                    msg.data = 1; //发布聚集开始信号
                    pub_start.publish(msg);
                    
                }
                if(replan_flag) //出现   特殊情况  重新规划路径
                {
                    if (!publish_goal) {
                        ROS_WARN("[FM2 Gather] Strategic mode active, convert robot-level replan signal into gather-center recompute.");
                        start_compute = true;
                        replan_flag = false;
                        std::fill(robot_replan_flags.begin(), robot_replan_flags.end(), false);
                        rate.sleep();
                        continue;
                    }
                    const double staging_radius = resolve_goal_occupied_staging_radius(hypotenuse);
                    for(int i = 0 ; i < num_robots ; i++)
                    {
                        if(robot_replan_flags[i])
                        {
                            if (shape_active_robot_ids.find(robot_ids[i]) != shape_active_robot_ids.end()) {
                                ROS_INFO("[FM2 Gather] Skip replan for robot %d because shape assembly is active.", robot_ids[i]);
                                robot_replan_flags[i] = false;
                                continue;
                            }
                            ROS_INFO("Replan for robot %d", robot_ids[i]);
                            if (i < static_cast<int>(goals.size())) {
                                bool handled = false;
                                if (publish_center_goal_only && i < static_cast<int>(robot_poses.size())) {
                                    handled = publish_goal_occupied_staging_goal(
                                        controllers[i],
                                        goals[i],
                                        robot_poses[i],
                                        i,
                                        num_robots,
                                        staging_radius);
                                }
                                if (!handled) {
                                    publish_random_goal(controllers[i], goals[i]);
                                }
                            } else {
                                ROS_WARN("Skip replan for robot %d due to goal index out of range", i + 1);
                            }
                            robot_replan_flags[i] = false;
                        }
                    }
                    replan_flag = false;
                }
            }
        }
        catch (tf::TransformException& ex)
        {
            ROS_WARN("%s", ex.what());
        }

        // 控制循环频率
        rate.sleep();
    }
    ros::shutdown();
    return 0;
}
