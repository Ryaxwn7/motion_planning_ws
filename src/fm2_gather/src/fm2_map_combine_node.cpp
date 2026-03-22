#include <ros/ros.h>
#include <ros/master.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <vector>
#include <string>
#include <deque>
#include <mutex>
#include <map>
#include <set>
#include <sstream>
#include <algorithm>
#include <cctype>
#include <cmath>

#include "fm2_gather/map.h"

namespace {

struct RobotCostmap {
    nav_msgs::OccupancyGrid data;
    ros::Time stamp;
    bool ready = false;
};

int g_num_robots = 0;
double g_resolution = -1.0;
int g_size_x = 0;
int g_size_y = 0;
int32_t g_cell_count = 0;
geometry_msgs::Pose g_origin;
bool g_map_initialized = false;

fm2_gather::map g_combined_map;
nav_msgs::OccupancyGrid g_background_map;
std::map<int, RobotCostmap> g_robot_costmaps;
std::map<int, std::deque<geometry_msgs::PoseStamped>> g_position_cache;
std::mutex g_costmap_mutex;

std::vector<int> g_robot_ids;
std::string g_background_map_topic = "/map";
std::string g_robot_costmap_topic_suffix = "move_base/global_costmap/costmap";
std::string g_robot_costmap_update_topic_suffix = "move_base/global_costmap/costmap_updates";
std::string g_robot_base_frame_suffix = "base_footprint";
std::string g_combined_map_topic = "/combined_map";
std::string g_combined_occupancy_topic = "/combined_map_occ";
int g_obstacle_threshold = 80;
bool g_unknown_is_obstacle = false;
double g_costmap_timeout = 1.0;
double g_robot_clear_radius = 0.35;
int g_robot_clear_history_size = 5;
double g_loop_hz = 10.0;
bool g_publish_combined_occupancy = false;

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

bool resolveRobotConfig(ros::NodeHandle& nh, int& resolved_num, std::vector<int>& resolved_ids)
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
            ROS_ERROR("[MAP COMBINE] auto_detect_robots=true but no robots were detected within %.2fs on %s*/%s",
                      robot_detect_timeout,
                      robot_namespace_prefix.c_str(),
                      robot_detect_topic_suffix.c_str());
            return false;
        }

        resolved_ids = detected_ids;
        resolved_num = static_cast<int>(resolved_ids.size());
        ROS_INFO("[MAP COMBINE] Auto-detected %d robots from topic suffix %s",
                 resolved_num, robot_detect_topic_suffix.c_str());
    } else {
        const bool has_num_param = nh.getParam("num_robots", resolved_num);
        nh.getParam("robot_ids", resolved_ids);

        if (has_num_param && resolved_num > 0) {
            if (resolved_ids.size() != static_cast<std::size_t>(resolved_num)) {
                resolved_ids.clear();
                resolved_ids.reserve(resolved_num);
                for (int i = 1; i <= resolved_num; ++i) {
                    resolved_ids.push_back(i);
                }
                ROS_WARN("[MAP COMBINE] robot_ids mismatched configured num_robots=%d, regenerate sequential IDs 1..%d",
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
        ROS_WARN("[MAP COMBINE] robot_ids missing, fallback to sequential IDs 1..%d", resolved_num);
    }
    if (resolved_num != static_cast<int>(resolved_ids.size())) {
        ROS_WARN("[MAP COMBINE] num_robots=%d mismatches robot_ids size=%zu, use robot_ids size",
                 resolved_num, resolved_ids.size());
        resolved_num = static_cast<int>(resolved_ids.size());
    }
    if (resolved_num <= 0 || resolved_ids.empty()) {
        ROS_ERROR("[MAP COMBINE] No valid robot set found, please check robot namespace/topic settings");
        return false;
    }

    nh.setParam("/num_robots", resolved_num);
    nh.setParam("/robot_ids", resolved_ids);
    nh.setParam("num_robots", resolved_num);
    nh.setParam("robot_ids", resolved_ids);
    return true;
}

bool isObstacleValue(const int value)
{
    if (value < 0) {
        return g_unknown_is_obstacle;
    }
    return value >= g_obstacle_threshold;
}

bool worldToMap(const double wx, const double wy, int& mx, int& my)
{
    if (!g_map_initialized || g_resolution <= 0.0) {
        return false;
    }
    if (wx < g_origin.position.x || wy < g_origin.position.y) {
        return false;
    }
    mx = static_cast<int>(std::round((wx - g_origin.position.x) / g_resolution));
    my = static_cast<int>(std::round((wy - g_origin.position.y) / g_resolution));
    if (mx < 0 || my < 0 || mx >= g_size_x || my >= g_size_y) {
        return false;
    }
    return true;
}

void backgroundMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    g_background_map = *msg;
    g_resolution = msg->info.resolution;
    g_size_x = static_cast<int>(msg->info.width);
    g_size_y = static_cast<int>(msg->info.height);
    g_cell_count = g_size_x * g_size_y;
    g_origin = msg->info.origin;
    g_map_initialized = (g_size_x > 0 && g_size_y > 0 && g_resolution > 0.0);
}

void robotCostmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg, int robot_id)
{
    std::lock_guard<std::mutex> lock(g_costmap_mutex);
    RobotCostmap& costmap = g_robot_costmaps[robot_id];
    costmap.data = *msg;
    costmap.stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    costmap.ready = true;
}

void initializeRobotCostmapStorage(RobotCostmap& costmap)
{
    costmap.data.header.frame_id = "map";
    costmap.data.info.resolution = g_resolution;
    costmap.data.info.width = static_cast<uint32_t>(g_size_x);
    costmap.data.info.height = static_cast<uint32_t>(g_size_y);
    costmap.data.info.origin = g_origin;
    costmap.data.data.assign(g_cell_count, 0);
}

void robotCostmapUpdateCallback(const map_msgs::OccupancyGridUpdate::ConstPtr& msg, int robot_id)
{
    std::lock_guard<std::mutex> lock(g_costmap_mutex);
    if (!g_map_initialized || g_cell_count <= 0) {
        ROS_WARN_THROTTLE(1.0, "[MAP COMBINE] Skip robot%d costmap update before background map is ready", robot_id);
        return;
    }

    RobotCostmap& costmap = g_robot_costmaps[robot_id];
    if (static_cast<int>(costmap.data.data.size()) != g_cell_count) {
        initializeRobotCostmapStorage(costmap);
    }

    const int x0 = msg->x;
    const int y0 = msg->y;
    const int width = static_cast<int>(msg->width);
    const int height = static_cast<int>(msg->height);
    if (width <= 0 || height <= 0) {
        return;
    }
    if (static_cast<int>(msg->data.size()) < width * height) {
        ROS_WARN_THROTTLE(1.0,
                          "[MAP COMBINE] robot%d invalid update payload size=%zu expected=%d",
                          robot_id,
                          msg->data.size(),
                          width * height);
        return;
    }

    for (int dy = 0; dy < height; ++dy) {
        const int y = y0 + dy;
        if (y < 0 || y >= g_size_y) {
            continue;
        }
        for (int dx = 0; dx < width; ++dx) {
            const int x = x0 + dx;
            if (x < 0 || x >= g_size_x) {
                continue;
            }
            const int dst_idx = y * g_size_x + x;
            const int src_idx = dy * width + dx;
            costmap.data.data[dst_idx] = msg->data[src_idx];
        }
    }

    costmap.stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    costmap.ready = true;
}

bool getRobotPoses(const std::vector<int>& robot_ids,
                   std::vector<geometry_msgs::PoseStamped>& robot_poses,
                   tf::TransformListener& pose_listener)
{
    robot_poses.clear();
    robot_poses.reserve(robot_ids.size());

    for (const int robot_id : robot_ids) {
        tf::StampedTransform transform;
        const std::string frame_id =
            "robot" + std::to_string(robot_id) + "/" + g_robot_base_frame_suffix;
        try {
            pose_listener.waitForTransform("map", frame_id, ros::Time(0), ros::Duration(0.3));
            pose_listener.lookupTransform("map", frame_id, ros::Time(0), transform);
        } catch (tf::TransformException& ex) {
            ROS_WARN_THROTTLE(1.0,
                              "[MAP COMBINE] TF lookup failed for robot%d frame=%s: %s",
                              robot_id,
                              frame_id.c_str(),
                              ex.what());
            return false;
        }

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
        robot_poses.push_back(pose_stamped);
    }

    return true;
}

void updatePositionCache(const std::vector<int>& robot_ids,
                         const std::vector<geometry_msgs::PoseStamped>& robot_poses)
{
    const std::size_t count = std::min(robot_ids.size(), robot_poses.size());
    for (std::size_t i = 0; i < count; ++i) {
        auto& cache = g_position_cache[robot_ids[i]];
        cache.push_back(robot_poses[i]);
        while (static_cast<int>(cache.size()) > std::max(1, g_robot_clear_history_size)) {
            cache.pop_front();
        }
    }
}

void initializeCombinedMapFromBackground()
{
    g_combined_map.gridSize.clear();
    g_combined_map.gridSize.push_back(static_cast<int16_t>(g_size_x));
    g_combined_map.gridSize.push_back(static_cast<int16_t>(g_size_y));
    g_combined_map.resolution = static_cast<float>(g_resolution);
    g_combined_map.origin = g_origin;
    g_combined_map.ndims = 2;
    g_combined_map.occupancyGrid.resize(g_cell_count, true);
}

void resetCombinedMapToBackground()
{
    if (!g_map_initialized || static_cast<int>(g_background_map.data.size()) != g_cell_count) {
        return;
    }
    if (static_cast<int>(g_combined_map.occupancyGrid.size()) != g_cell_count) {
        initializeCombinedMapFromBackground();
    }
    for (int i = 0; i < g_cell_count; ++i) {
        g_combined_map.occupancyGrid[i] = (g_background_map.data[i] != 100);
    }
}

void overlayRobotCostmaps()
{
    const ros::Time now = ros::Time::now();
    std::lock_guard<std::mutex> lock(g_costmap_mutex);
    for (const int robot_id : g_robot_ids) {
        auto it = g_robot_costmaps.find(robot_id);
        if (it == g_robot_costmaps.end() || !it->second.ready) {
            continue;
        }

        const RobotCostmap& cmap = it->second;
        if (g_costmap_timeout > 0.0) {
            const double age = (now - cmap.stamp).toSec();
            if (age > g_costmap_timeout) {
                ROS_WARN_THROTTLE(1.0,
                                  "[MAP COMBINE] robot%d costmap stale age=%.2fs timeout=%.2fs, skip",
                                  robot_id,
                                  age,
                                  g_costmap_timeout);
                continue;
            }
        }

        if (static_cast<int>(cmap.data.data.size()) != g_cell_count) {
            ROS_WARN_THROTTLE(1.0,
                              "[MAP COMBINE] robot%d costmap size mismatch data=%zu expected=%d, skip",
                              robot_id,
                              cmap.data.data.size(),
                              g_cell_count);
            continue;
        }
        if (static_cast<int>(cmap.data.info.width) != g_size_x ||
            static_cast<int>(cmap.data.info.height) != g_size_y) {
            ROS_WARN_THROTTLE(1.0,
                              "[MAP COMBINE] robot%d costmap geometry mismatch %ux%u expected %dx%d, skip",
                              robot_id,
                              cmap.data.info.width,
                              cmap.data.info.height,
                              g_size_x,
                              g_size_y);
            continue;
        }

        for (int idx = 0; idx < g_cell_count; ++idx) {
            if (isObstacleValue(static_cast<int>(cmap.data.data[idx]))) {
                g_combined_map.occupancyGrid[idx] = false;
            }
        }
    }
}

void clearRobotFootprintsFromCombinedMap()
{
    if (!g_map_initialized || g_resolution <= 0.0) {
        return;
    }

    const int radius_cells = std::max(
        0,
        static_cast<int>(std::ceil(g_robot_clear_radius / std::max(1e-6, g_resolution))));
    if (radius_cells <= 0) {
        return;
    }

    for (const auto& kv : g_position_cache) {
        const auto& poses = kv.second;
        for (const auto& pose : poses) {
            int mx = 0;
            int my = 0;
            if (!worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my)) {
                continue;
            }
            for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
                const int ny = my + dy;
                if (ny < 0 || ny >= g_size_y) {
                    continue;
                }
                for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
                    const int nx = mx + dx;
                    if (nx < 0 || nx >= g_size_x) {
                        continue;
                    }
                    if (dx * dx + dy * dy > radius_cells * radius_cells) {
                        continue;
                    }
                    g_combined_map.occupancyGrid[ny * g_size_x + nx] = true;
                }
            }
        }
    }
}

void publishCombinedAsOccupancyGrid(const ros::Publisher& publisher)
{
    if (!g_publish_combined_occupancy || publisher.getTopic().empty()) {
        return;
    }
    nav_msgs::OccupancyGrid map_msg;
    map_msg.header.stamp = ros::Time::now();
    map_msg.header.frame_id = "map";
    map_msg.info.resolution = g_resolution;
    map_msg.info.width = static_cast<uint32_t>(g_size_x);
    map_msg.info.height = static_cast<uint32_t>(g_size_y);
    map_msg.info.origin = g_origin;
    map_msg.data.resize(g_cell_count, 0);
    for (int i = 0; i < g_cell_count; ++i) {
        map_msg.data[i] = g_combined_map.occupancyGrid[i] ? 0 : 100;
    }
    publisher.publish(map_msg);
}

}  // namespace

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fm2_map_combine_node");
    ros::NodeHandle nh;

    if (!resolveRobotConfig(nh, g_num_robots, g_robot_ids)) {
        return 1;
    }

    nh.param("background_map_topic", g_background_map_topic, g_background_map_topic);
    nh.param("robot_costmap_topic_suffix", g_robot_costmap_topic_suffix, g_robot_costmap_topic_suffix);
    nh.param("robot_costmap_update_topic_suffix",
             g_robot_costmap_update_topic_suffix,
             g_robot_costmap_update_topic_suffix);
    nh.param("robot_base_frame_suffix", g_robot_base_frame_suffix, g_robot_base_frame_suffix);
    nh.param("combined_map_topic", g_combined_map_topic, g_combined_map_topic);
    nh.param("combined_occupancy_topic", g_combined_occupancy_topic, g_combined_occupancy_topic);
    nh.param("map_combine_obstacle_threshold", g_obstacle_threshold, g_obstacle_threshold);
    nh.param("map_combine_unknown_is_obstacle", g_unknown_is_obstacle, g_unknown_is_obstacle);
    nh.param("map_combine_costmap_timeout", g_costmap_timeout, g_costmap_timeout);
    nh.param("map_combine_robot_clear_radius", g_robot_clear_radius, g_robot_clear_radius);
    nh.param("map_combine_robot_clear_history_size", g_robot_clear_history_size, g_robot_clear_history_size);
    nh.param("map_combine_rate", g_loop_hz, g_loop_hz);
    nh.param("publish_combined_occupancy", g_publish_combined_occupancy, g_publish_combined_occupancy);

    while (!g_robot_costmap_topic_suffix.empty() && g_robot_costmap_topic_suffix.front() == '/') {
        g_robot_costmap_topic_suffix.erase(0, 1);
    }
    while (!g_robot_costmap_update_topic_suffix.empty() &&
           g_robot_costmap_update_topic_suffix.front() == '/') {
        g_robot_costmap_update_topic_suffix.erase(0, 1);
    }
    g_obstacle_threshold = std::max(0, std::min(100, g_obstacle_threshold));
    g_costmap_timeout = std::max(0.0, g_costmap_timeout);
    g_robot_clear_radius = std::max(0.0, g_robot_clear_radius);
    g_robot_clear_history_size = std::max(1, g_robot_clear_history_size);
    g_loop_hz = std::max(1.0, g_loop_hz);

    std::ostringstream ids_oss;
    for (std::size_t i = 0; i < g_robot_ids.size(); ++i) {
        if (i > 0) {
            ids_oss << ",";
        }
        ids_oss << g_robot_ids[i];
    }
    ROS_INFO("[MAP COMBINE] Resolved robots: num_robots=%d ids=[%s]",
             g_num_robots,
             ids_oss.str().c_str());
    ROS_INFO("[MAP COMBINE] background_map_topic=%s robot_costmap_topic_suffix=%s combined_map_topic=%s",
             g_background_map_topic.c_str(),
             g_robot_costmap_topic_suffix.c_str(),
             g_combined_map_topic.c_str());

    ros::Subscriber bg_sub = nh.subscribe<nav_msgs::OccupancyGrid>(
        g_background_map_topic,
        1,
        backgroundMapCallback);

    std::vector<ros::Subscriber> costmap_subs;
    costmap_subs.reserve(g_num_robots);
    std::vector<ros::Subscriber> costmap_update_subs;
    costmap_update_subs.reserve(g_num_robots);
    for (const int robot_id : g_robot_ids) {
        const std::string topic =
            "/robot" + std::to_string(robot_id) + "/" + g_robot_costmap_topic_suffix;
        const std::string update_topic =
            "/robot" + std::to_string(robot_id) + "/" + g_robot_costmap_update_topic_suffix;
        ROS_INFO("[MAP COMBINE] Subscribing robot costmap topic: %s", topic.c_str());
        costmap_subs.push_back(
            nh.subscribe<nav_msgs::OccupancyGrid>(
                topic,
                1,
                boost::bind(robotCostmapCallback, _1, robot_id)));
        ROS_INFO("[MAP COMBINE] Subscribing robot costmap update topic: %s", update_topic.c_str());
        costmap_update_subs.push_back(
            nh.subscribe<map_msgs::OccupancyGridUpdate>(
                update_topic,
                10,
                boost::bind(robotCostmapUpdateCallback, _1, robot_id)));
    }

    ros::Publisher combined_map_pub =
        nh.advertise<fm2_gather::map>(g_combined_map_topic, 1, true);
    ros::Publisher combined_occ_pub;
    if (g_publish_combined_occupancy) {
        combined_occ_pub = nh.advertise<nav_msgs::OccupancyGrid>(g_combined_occupancy_topic, 1, true);
    }

    tf::TransformListener tf_listener;
    ros::Rate rate(g_loop_hz);

    while (ros::ok()) {
        ros::spinOnce();

        if (g_map_initialized && static_cast<int>(g_combined_map.occupancyGrid.size()) != g_cell_count) {
            initializeCombinedMapFromBackground();
        }

        if (g_map_initialized) {
            std::vector<geometry_msgs::PoseStamped> robot_poses;
            if (getRobotPoses(g_robot_ids, robot_poses, tf_listener)) {
                updatePositionCache(g_robot_ids, robot_poses);
            }

            resetCombinedMapToBackground();
            overlayRobotCostmaps();
            clearRobotFootprintsFromCombinedMap();
            combined_map_pub.publish(g_combined_map);
            publishCombinedAsOccupancyGrid(combined_occ_pub);
        }

        rate.sleep();
    }

    return 0;
}
