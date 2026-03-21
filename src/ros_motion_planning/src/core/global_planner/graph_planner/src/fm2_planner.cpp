/*

FM2_Planner.cpp

Author: YXW

Date: 2024/11/13

Description: FM2 路径规划器

*/

#include "fm2_planner.h"

#include <algorithm>
#include <cmath>
#include <fstream>

#define LETHAL_COST 253

static void plotUncertaintyVelocityAndTimeWindow(
    const std::vector<double>& velocity_map,
    nDGridMap<FMCell, 2>& grid,
    const std::string& window_title,
    bool flipY = true);

namespace global_planner {

namespace {
const char kDefaultVelocityPath[] = "/tmp/fm2_velocity.csv";

long long makeKernelKey(const int hard_cells, const int soft_cells) {
    return (static_cast<long long>(hard_cells) << 32) |
           static_cast<unsigned int>(soft_cells);
}

double clampDouble(const double value, const double min_v, const double max_v) {
    return std::max(min_v, std::min(value, max_v));
}

int clampInt(const int value, const int min_v, const int max_v) {
    return std::max(min_v, std::min(value, max_v));
}
}  // namespace

FM2_Planner::FM2_Planner(costmap_2d::Costmap2D* costmap) : GlobalPlanner(costmap) {
    ros::NodeHandle nh("~");
    loadParams(nh);
    setupDynamicReconfigure(nh);
    initializeFastMarching(costmap);
}

FM2_Planner::FM2_Planner(costmap_2d::Costmap2DROS* costmap_ros, costmap_2d::Costmap2D* costmap) : GlobalPlanner(costmap) {
    ros::NodeHandle nh("~");
    loadParams(nh);
    setupDynamicReconfigure(nh);
    initializeFastMarchingROS(costmap_ros, costmap);
}

FM2_Planner::FM2_Planner(costmap_2d::Costmap2DROS* costmap_ros,
                         costmap_2d::Costmap2D* costmap,
                         const ros::NodeHandle& nh) : GlobalPlanner(costmap) {
    loadParams(nh);
    setupDynamicReconfigure(nh);
    initializeFastMarchingROS(costmap_ros, costmap);
}

void FM2_Planner::loadParams(const ros::NodeHandle& nh) {
    // 与 fm2_gather 参数保持一致
    nh.param("fm2/v_max", v_max_, 1.0);
    nh.param("fm2/velocity_alpha", velocity_alpha_, 0.2);
    nh.param("fm2/velocity_dmax", velocity_dmax_, -1.0);
    nh.param("fm2/robot_radius", robot_radius_, 0.0);
    nh.param("fm2/velocity_mode", velocity_mode_, 0);
    nh.param("fm2/velocity_sigmoid_k", velocity_sigmoid_k_, 0.15);
    nh.param("fm2/velocity_sigmoid_b", velocity_sigmoid_b_, 0.0);
    nh.param("fm2/use_gather_style", use_gather_style_, true);

    nh.param("fm2/use_dynamic_obstacle_uncertainty", use_dynamic_obstacle_uncertainty_, true);
    nh.param("fm2/use_dynamic_obstacle_cluster_tracking", use_dynamic_obstacle_cluster_tracking_, true);
    nh.param("fm2/force_full_occupancy_refresh", force_full_occupancy_refresh_, false);
    nh.param("fm2/dynamic_obstacle_threshold", dynamic_obstacle_threshold_, LETHAL_COST);
    nh.param("fm2/dynamic_obstacle_unknown_is_obstacle", dynamic_obstacle_unknown_is_obstacle_, false);
    nh.param("fm2/dynamic_obstacle_inflation_radius", dynamic_obstacle_inflation_radius_, 0.2);
    nh.param("fm2/dynamic_obstacle_uncertainty_radius", dynamic_obstacle_uncertainty_radius_, 0.6);
    nh.param("fm2/dynamic_obstacle_stride", dynamic_obstacle_stride_, 1);
    nh.param("fm2/dynamic_obstacle_cluster_min_cells", dynamic_obstacle_cluster_min_cells_, 2);
    nh.param("fm2/dynamic_obstacle_track_match_radius", dynamic_obstacle_track_match_radius_, 0.5);
    nh.param("fm2/dynamic_obstacle_track_ttl", dynamic_obstacle_track_ttl_, 0.6);
    nh.param("fm2/dynamic_obstacle_max_samples", dynamic_obstacle_max_samples_, 800);
    nh.param("fm2/static_obstacle_noise_reject_cells", static_obstacle_noise_reject_cells_, 4);

    nh.param("fm2/debug_dump_velocity", debug_dump_velocity_, false);
    nh.param("fm2/debug_visualize", debug_visualize_, false);
    nh.param("fm2/debug_log", debug_log_, false);
    nh.param("fm2/debug_velocity_path", debug_velocity_path_, std::string(kDefaultVelocityPath));

    v_max_ = clampDouble(v_max_, 0.0, 5.0);
    velocity_alpha_ = clampDouble(velocity_alpha_, 0.0, 1.0);
    velocity_dmax_ = clampDouble(velocity_dmax_, 0.0, 5.0);
    robot_radius_ = clampDouble(robot_radius_, 0.0, 3.0);
    velocity_mode_ = clampInt(velocity_mode_, 0, 1);
    velocity_sigmoid_k_ = clampDouble(velocity_sigmoid_k_, 0.0, 10.0);
    velocity_sigmoid_b_ = clampDouble(velocity_sigmoid_b_, 0.0, 10.0);

    dynamic_obstacle_threshold_ = clampInt(dynamic_obstacle_threshold_, 0, 255);
    dynamic_obstacle_inflation_radius_ = clampDouble(dynamic_obstacle_inflation_radius_, 0.0, 5.0);
    dynamic_obstacle_uncertainty_radius_ =
        std::max(dynamic_obstacle_inflation_radius_,
                 clampDouble(dynamic_obstacle_uncertainty_radius_, 0.0, 10.0));
    dynamic_obstacle_stride_ = std::max(1, dynamic_obstacle_stride_);
    dynamic_obstacle_cluster_min_cells_ = std::max(1, dynamic_obstacle_cluster_min_cells_);
    dynamic_obstacle_track_match_radius_ = clampDouble(dynamic_obstacle_track_match_radius_, 0.0, 10.0);
    dynamic_obstacle_track_ttl_ = clampDouble(dynamic_obstacle_track_ttl_, 0.0, 10.0);
    dynamic_obstacle_max_samples_ = std::max(1, dynamic_obstacle_max_samples_);
    static_obstacle_noise_reject_cells_ = std::max(0, static_obstacle_noise_reject_cells_);
}

void FM2_Planner::setupDynamicReconfigure(const ros::NodeHandle& nh) {
    ros::NodeHandle fm2_nh(nh, "fm2");
    dyn_server_ = new dynamic_reconfigure::Server<graph_planner::FM2PlannerConfig>(fm2_nh);
    dyn_cb_ = boost::bind(&FM2_Planner::reconfigureCB, this, _1, _2);
    dyn_server_->setCallback(dyn_cb_);
}

void FM2_Planner::reconfigureCB(graph_planner::FM2PlannerConfig& config, uint32_t level) {
    if (!setup_) {
        default_config_ = config;
        setup_ = true;
    }

    if (config.restore_defaults) {
        config = default_config_;
        config.restore_defaults = false;
    }

    config.v_max = clampDouble(config.v_max, 0.0, 5.0);
    config.velocity_alpha = clampDouble(config.velocity_alpha, 0.0, 1.0);
    config.velocity_dmax = clampDouble(config.velocity_dmax, 0.0, 5.0);
    config.robot_radius = clampDouble(config.robot_radius, 0.0, 3.0);
    config.velocity_mode = clampInt(config.velocity_mode, 0, 1);
    config.velocity_sigmoid_k = clampDouble(config.velocity_sigmoid_k, 0.0, 10.0);
    config.velocity_sigmoid_b = clampDouble(config.velocity_sigmoid_b, 0.0, 10.0);

    config.dynamic_obstacle_threshold = clampInt(config.dynamic_obstacle_threshold, 0, 255);
    config.dynamic_obstacle_inflation_radius = clampDouble(config.dynamic_obstacle_inflation_radius, 0.0, 5.0);
    config.dynamic_obstacle_uncertainty_radius =
        std::max(config.dynamic_obstacle_inflation_radius,
                 clampDouble(config.dynamic_obstacle_uncertainty_radius, 0.0, 10.0));
    config.dynamic_obstacle_stride = std::max(1, config.dynamic_obstacle_stride);
    config.dynamic_obstacle_cluster_min_cells = std::max(1, config.dynamic_obstacle_cluster_min_cells);
    config.dynamic_obstacle_track_match_radius =
        clampDouble(config.dynamic_obstacle_track_match_radius, 0.0, 10.0);
    config.dynamic_obstacle_track_ttl =
        clampDouble(config.dynamic_obstacle_track_ttl, 0.0, 10.0);
    config.static_obstacle_noise_reject_cells = std::max(0, config.static_obstacle_noise_reject_cells);
    config.dynamic_obstacle_max_samples = std::max(1, config.dynamic_obstacle_max_samples);

    v_max_ = config.v_max;
    velocity_alpha_ = config.velocity_alpha;
    velocity_dmax_ = config.velocity_dmax;
    robot_radius_ = config.robot_radius;
    velocity_mode_ = config.velocity_mode;
    velocity_sigmoid_k_ = config.velocity_sigmoid_k;
    velocity_sigmoid_b_ = config.velocity_sigmoid_b;
    use_gather_style_ = config.use_gather_style;

    use_dynamic_obstacle_uncertainty_ = config.use_dynamic_obstacle_uncertainty;
    use_dynamic_obstacle_cluster_tracking_ = config.use_dynamic_obstacle_cluster_tracking;
    force_full_occupancy_refresh_ = config.force_full_occupancy_refresh;
    dynamic_obstacle_threshold_ = config.dynamic_obstacle_threshold;
    dynamic_obstacle_unknown_is_obstacle_ = config.dynamic_obstacle_unknown_is_obstacle;
    dynamic_obstacle_inflation_radius_ = config.dynamic_obstacle_inflation_radius;
    dynamic_obstacle_uncertainty_radius_ = config.dynamic_obstacle_uncertainty_radius;
    dynamic_obstacle_stride_ = config.dynamic_obstacle_stride;
    dynamic_obstacle_cluster_min_cells_ = config.dynamic_obstacle_cluster_min_cells;
    dynamic_obstacle_track_match_radius_ = config.dynamic_obstacle_track_match_radius;
    dynamic_obstacle_track_ttl_ = config.dynamic_obstacle_track_ttl;
    static_obstacle_noise_reject_cells_ = config.static_obstacle_noise_reject_cells;
    dynamic_obstacle_max_samples_ = config.dynamic_obstacle_max_samples;

    debug_dump_velocity_ = config.debug_dump_velocity;
    debug_visualize_ = config.debug_visualize;
    debug_log_ = config.debug_log;
    if (!config.debug_velocity_path.empty()) {
        debug_velocity_path_ = config.debug_velocity_path;
    } else if (debug_velocity_path_.empty()) {
        debug_velocity_path_ = kDefaultVelocityPath;
    }

    rebuildStaticObstacleProximityMask();
    invalidateVelocityCache();

    if (debug_log_) {
        ROS_INFO("FM2: Reconfigure v_max=%.3f alpha=%.3f dmax=%.3f radius=%.3f mode=%d k=%.3f b=%.3f gather=%s dyn_unc=%s cluster_track=%s full_refresh=%s cluster_min=%d track_r=%.2f ttl=%.2f static_reject=%d",
                 v_max_, velocity_alpha_, velocity_dmax_, robot_radius_, velocity_mode_,
                 velocity_sigmoid_k_, velocity_sigmoid_b_, use_gather_style_ ? "true" : "false",
                 use_dynamic_obstacle_uncertainty_ ? "true" : "false",
                 use_dynamic_obstacle_cluster_tracking_ ? "true" : "false",
                 force_full_occupancy_refresh_ ? "true" : "false",
                 dynamic_obstacle_cluster_min_cells_,
                 dynamic_obstacle_track_match_radius_,
                 dynamic_obstacle_track_ttl_,
                 static_obstacle_noise_reject_cells_);
    }
}

void FM2_Planner::dumpVelocityMapCsv(const std::string& path) {
    std::ofstream out(path.c_str());
    if (!out.is_open()) {
        return;
    }
    out << "x,y,velocity\n";
    for (int y = 0; y < ny_; ++y) {
        for (int x = 0; x < nx_; ++x) {
            const int idx = y * nx_ + x;
            out << x << "," << y << "," << grid_.getCell(idx).getVelocity() << "\n";
        }
    }
}

void FM2_Planner::initializeFastMarching(costmap_2d::Costmap2D* costmap) {
    ndims_ = 2;
    nx_ = costmap->getSizeInCellsX();
    ny_ = costmap->getSizeInCellsY();
    ns_ = nx_ * ny_;
    resolution_ = costmap->getResolution();
    global_costmap_ = costmap->getCharMap();
    static_costmap_ = global_costmap_;
    obstacle_costmap_ = nullptr;
    lethal_cost_ = LETHAL_COST;
    obstacle_layer_ = nullptr;

    dimsize_ = {nx_, ny_};
    grid_.resize(dimsize_);
    grid_.setLeafSize(resolution_);

    fm2_sources_.clear();
    for (int i = 0; i < ns_; ++i) {
        const bool is_occupied =
            (global_costmap_[i] >= lethal_cost_) || (global_costmap_[i] == costmap_2d::NO_INFORMATION);
        grid_.getCell(i).setOccupancy(!is_occupied);
        if (is_occupied) {
            fm2_sources_.push_back(i);
        }
    }

    rebuildStaticObstacleProximityMask();
    invalidateVelocityCache();

    if (debug_visualize_) {
        GridPlotter::plotMap(grid_);
    }
}

void FM2_Planner::initializeFastMarchingROS(costmap_2d::Costmap2DROS* costmap_ros, costmap_2d::Costmap2D* costmap) {
    ndims_ = 2;
    nx_ = costmap->getSizeInCellsX();
    ny_ = costmap->getSizeInCellsY();
    ns_ = nx_ * ny_;
    resolution_ = costmap->getResolution();
    global_costmap_ = costmap->getCharMap();
    lethal_cost_ = LETHAL_COST;

    bool static_layer_exist = false;
    bool obstacle_layer_exist = false;
    obstacle_layer_ = nullptr;
    static_costmap_ = nullptr;
    obstacle_costmap_ = nullptr;

    for (auto layer = costmap_ros->getLayeredCostmap()->getPlugins()->begin();
         layer != costmap_ros->getLayeredCostmap()->getPlugins()->end(); ++layer)
    {
        if (!obstacle_layer_exist) {
            boost::shared_ptr<costmap_2d::ObstacleLayer> obstacle_layer =
                boost::dynamic_pointer_cast<costmap_2d::ObstacleLayer>(*layer);
            if (obstacle_layer) {
                obstacle_layer_exist = true;
                obstacle_layer_ = obstacle_layer.get();
                boost::unique_lock<boost::recursive_mutex> lock(*(obstacle_layer_->getMutex()));
                obstacle_costmap_ = obstacle_layer_->getCharMap();
            }
        }

        if (!static_layer_exist) {
            boost::shared_ptr<costmap_2d::StaticLayer> static_layer =
                boost::dynamic_pointer_cast<costmap_2d::StaticLayer>(*layer);
            if (static_layer) {
                static_layer_exist = true;
                boost::unique_lock<boost::recursive_mutex> lock(*(static_layer->getMutex()));
                static_costmap_ = static_layer->getCharMap();
            }
        }
    }

    if (!static_layer_exist || static_costmap_ == nullptr) {
        ROS_WARN("FM2: static layer not found, fallback to global costmap as occupancy initialization");
        static_costmap_ = global_costmap_;
    }
    if (!obstacle_layer_exist || obstacle_layer_ == nullptr) {
        ROS_WARN("FM2: obstacle layer not found, dynamic uncertainty layer will fallback to merged costmap");
    }

    dimsize_ = {nx_, ny_};
    grid_.resize(dimsize_);
    grid_.setLeafSize(resolution_);

    fm2_sources_.clear();
    for (int i = 0; i < ns_; ++i) {
        const bool is_occupied =
            (static_costmap_[i] >= lethal_cost_) || (static_costmap_[i] == costmap_2d::NO_INFORMATION);
        grid_.getCell(i).setOccupancy(!is_occupied);
        if (is_occupied) {
            fm2_sources_.push_back(i);
        }
    }

    rebuildStaticObstacleProximityMask();
    invalidateVelocityCache();

    if (debug_visualize_) {
        GridPlotter::plotMap(grid_);
    }
}

bool FM2_Planner::recoverGrid() {
    int i = 0;
    for (i = 0; i < grid_.size(); ++i)
    {
        if (!grid_.getCell(i).isOccupied())
        {
            grid_.getCell(i).setValue(std::numeric_limits<double>::infinity());
            grid_.getCell(i).setVelocity(1.0);
            grid_.getCell(i).setState(FMState::OPEN);
        }
    }
    return i == grid_.size();
}

bool FM2_Planner::resetGrid() {
    for (int i = 0; i < ns_; ++i)
    {
        grid_.getCell(i).setOccupancy(true);
        grid_.getCell(i).setValue(std::numeric_limits<double>::infinity());
        grid_.getCell(i).setVelocity(1.0);
        grid_.getCell(i).setState(FMState::OPEN);
    }
    return true;
}

bool FM2_Planner::updateGrid(costmap_2d::Costmap2D* costmap)
{
    if (debug_log_) {
        ROS_INFO("FM2: Updating occupancy grid from costmap (full refresh)");
    }
    if (!resetGrid())
    {
        ROS_ERROR("FM2: resetGrid failed");
        return false;
    }
    fm2_sources_.clear();
    for (int i = 0; i < ns_; ++i) {
        const bool is_occupied =
            (costmap->getCharMap()[i] >= lethal_cost_) || (costmap->getCharMap()[i] == costmap_2d::NO_INFORMATION);
        grid_.getCell(i).setOccupancy(!is_occupied);
        if (is_occupied) {
            fm2_sources_.push_back(i);
        }
    }
    if (static_costmap_ == global_costmap_) {
        rebuildStaticObstacleProximityMask();
    }
    invalidateVelocityCache();
    return true;
}

void FM2_Planner::invalidateVelocityCache() {
    base_velocity_map_.clear();
    base_velocity_ready_ = false;
    uncertainty_kernel_cache_.clear();
    uncertainty_kernel_resolution_ = -1.0;
    dynamic_obstacle_tracks_.clear();
}

void FM2_Planner::rebuildStaticObstacleProximityMask() {
    static_obstacle_proximity_mask_.assign(static_cast<std::size_t>(std::max(0, ns_)), 0);
    if (ns_ <= 0 || nx_ <= 0 || ny_ <= 0 || static_costmap_ == nullptr) {
        return;
    }

    const int reject_cells = std::max(0, static_obstacle_noise_reject_cells_);
    std::vector<std::array<int, 2>> offsets;
    offsets.reserve(static_cast<std::size_t>((2 * reject_cells + 1) * (2 * reject_cells + 1)));
    for (int dy = -reject_cells; dy <= reject_cells; ++dy) {
        for (int dx = -reject_cells; dx <= reject_cells; ++dx) {
            if ((dx * dx + dy * dy) <= (reject_cells * reject_cells)) {
                offsets.push_back({{dx, dy}});
            }
        }
    }
    if (offsets.empty()) {
        offsets.push_back({{0, 0}});
    }

    for (int y = 0; y < ny_; ++y) {
        const int row = y * nx_;
        for (int x = 0; x < nx_; ++x) {
            const int idx = row + x;
            if (static_costmap_[idx] < lethal_cost_) {
                continue;
            }
            for (const auto& off : offsets) {
                const int nx = x + off[0];
                const int ny = y + off[1];
                if (nx < 0 || nx >= nx_ || ny < 0 || ny >= ny_) {
                    continue;
                }
                static_obstacle_proximity_mask_[static_cast<std::size_t>(ny * nx_ + nx)] = 1;
            }
        }
    }
}

bool FM2_Planner::isNearStaticObstacle(const int idx) const {
    if (idx < 0 || idx >= ns_) {
        return false;
    }
    if (static_obstacle_proximity_mask_.size() == static_cast<std::size_t>(ns_)) {
        return static_obstacle_proximity_mask_[static_cast<std::size_t>(idx)] != 0;
    }
    return (static_costmap_ != nullptr) && (static_costmap_[idx] >= lethal_cost_);
}

bool FM2_Planner::isDynamicObstacleCost(const unsigned char cost) const {
    if (cost == costmap_2d::NO_INFORMATION) {
        return dynamic_obstacle_unknown_is_obstacle_;
    }
    return static_cast<int>(cost) >= dynamic_obstacle_threshold_;
}

void FM2_Planner::collectDynamicObstacleCells(
    const unsigned char* map_data,
    std::vector<int>& occupied_indices) const {
    occupied_indices.clear();
    if (map_data == nullptr || ns_ <= 0) {
        return;
    }

    const int stride = std::max(1, dynamic_obstacle_stride_);
    occupied_indices.reserve(ns_ / std::max(1, stride * stride * 4));
    for (int y = 0; y < ny_; y += stride) {
        const int row = y * nx_;
        for (int x = 0; x < nx_; x += stride) {
            const int idx = row + x;
            if (isNearStaticObstacle(idx)) {
                continue;
            }
            if (isDynamicObstacleCost(map_data[idx])) {
                occupied_indices.push_back(idx);
            }
        }
    }
}

void FM2_Planner::extractDynamicObstacleClusters(
    const unsigned char* map_data,
    std::vector<DynamicObstacleCluster>& clusters) const {
    clusters.clear();
    if (map_data == nullptr || ns_ <= 0) {
        return;
    }

    const int stride = std::max(1, dynamic_obstacle_stride_);
    const int min_cells = std::max(1, dynamic_obstacle_cluster_min_cells_);
    const int neighbor_step = std::max(1, stride);
    std::vector<uint8_t> candidate_mask(static_cast<std::size_t>(ns_), 0);
    std::vector<uint8_t> visited(static_cast<std::size_t>(ns_), 0);
    std::vector<int> candidate_indices;
    candidate_indices.reserve(ns_ / std::max(1, stride * stride * 4));

    for (int y = 0; y < ny_; y += stride) {
        const int row = y * nx_;
        for (int x = 0; x < nx_; x += stride) {
            const int idx = row + x;
            if (isNearStaticObstacle(idx)) {
                continue;
            }
            if (isDynamicObstacleCost(map_data[idx])) {
                candidate_mask[static_cast<std::size_t>(idx)] = 1;
                candidate_indices.push_back(idx);
            }
        }
    }

    std::vector<int> queue;
    for (const int seed : candidate_indices) {
        if (visited[static_cast<std::size_t>(seed)] != 0) {
            continue;
        }
        queue.clear();
        queue.push_back(seed);
        visited[static_cast<std::size_t>(seed)] = 1;

        int cells = 0;
        double sum_x = 0.0;
        double sum_y = 0.0;
        for (std::size_t head = 0; head < queue.size(); ++head) {
            const int idx = queue[head];
            const int cx = idx % nx_;
            const int cy = idx / nx_;
            ++cells;
            sum_x += static_cast<double>(cx);
            sum_y += static_cast<double>(cy);

            for (int dy = -neighbor_step; dy <= neighbor_step; dy += neighbor_step) {
                for (int dx = -neighbor_step; dx <= neighbor_step; dx += neighbor_step) {
                    if (dx == 0 && dy == 0) {
                        continue;
                    }
                    const int nx = cx + dx;
                    const int ny = cy + dy;
                    if (nx < 0 || nx >= nx_ || ny < 0 || ny >= ny_) {
                        continue;
                    }
                    const int nidx = ny * nx_ + nx;
                    if (candidate_mask[static_cast<std::size_t>(nidx)] == 0 ||
                        visited[static_cast<std::size_t>(nidx)] != 0) {
                        continue;
                    }
                    visited[static_cast<std::size_t>(nidx)] = 1;
                    queue.push_back(nidx);
                }
            }
        }

        if (cells < min_cells) {
            continue;
        }

        DynamicObstacleCluster cluster;
        cluster.x = sum_x / static_cast<double>(cells);
        cluster.y = sum_y / static_cast<double>(cells);
        cluster.cells = cells;
        clusters.push_back(cluster);
    }
}

void FM2_Planner::updateDynamicObstacleTracks(
    const std::vector<DynamicObstacleCluster>& clusters,
    const ros::Time& stamp) {
    const double ttl = std::max(0.0, dynamic_obstacle_track_ttl_);
    dynamic_obstacle_tracks_.erase(
        std::remove_if(
            dynamic_obstacle_tracks_.begin(),
            dynamic_obstacle_tracks_.end(),
            [&](const DynamicObstacleTrack& track) {
                return (stamp - track.last_seen).toSec() > ttl;
            }),
        dynamic_obstacle_tracks_.end());

    if (clusters.empty()) {
        return;
    }

    const double match_radius_cells =
        std::max(0.0, dynamic_obstacle_track_match_radius_ / std::max(1e-6, resolution_));
    const double match_radius_sq = match_radius_cells * match_radius_cells;
    std::vector<uint8_t> cluster_used(clusters.size(), 0);

    for (auto& track : dynamic_obstacle_tracks_) {
        int best_cluster = -1;
        double best_dist_sq = std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < clusters.size(); ++i) {
            if (cluster_used[i] != 0) {
                continue;
            }
            const double dx = clusters[i].x - track.x;
            const double dy = clusters[i].y - track.y;
            const double dist_sq = dx * dx + dy * dy;
            if (dist_sq > match_radius_sq || dist_sq >= best_dist_sq) {
                continue;
            }
            best_dist_sq = dist_sq;
            best_cluster = static_cast<int>(i);
        }

        if (best_cluster < 0) {
            continue;
        }

        const DynamicObstacleCluster& cluster = clusters[static_cast<std::size_t>(best_cluster)];
        track.x = cluster.x;
        track.y = cluster.y;
        track.cells = cluster.cells;
        track.last_seen = stamp;
        cluster_used[static_cast<std::size_t>(best_cluster)] = 1;
    }

    for (std::size_t i = 0; i < clusters.size(); ++i) {
        if (cluster_used[i] != 0) {
            continue;
        }
        DynamicObstacleTrack track;
        track.x = clusters[i].x;
        track.y = clusters[i].y;
        track.cells = clusters[i].cells;
        track.last_seen = stamp;
        dynamic_obstacle_tracks_.push_back(track);
    }
}

double FM2_Planner::getVelocityReference(const std::vector<double>& velocity_map) const {
    if (velocity_map.empty()) {
        return std::max(1e-3, v_max_);
    }
    const auto it = std::max_element(velocity_map.begin(), velocity_map.end());
    const double vmax_ref = (it == velocity_map.end()) ? 0.0 : *it;
    return (vmax_ref > 1e-6) ? vmax_ref : std::max(1e-3, v_max_);
}

const std::vector<FM2_Planner::UncertaintyKernelCell>&
FM2_Planner::getUncertaintyKernel(int hard_cells, int soft_cells) {
    hard_cells = std::max(0, hard_cells);
    soft_cells = std::max(hard_cells, soft_cells);

    if (uncertainty_kernel_resolution_ != resolution_) {
        uncertainty_kernel_cache_.clear();
        uncertainty_kernel_resolution_ = resolution_;
    }

    const long long key = makeKernelKey(hard_cells, soft_cells);
    auto it = uncertainty_kernel_cache_.find(key);
    if (it != uncertainty_kernel_cache_.end()) {
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
                ratio = clampDouble(ratio, 0.0, 1.0);
            } else {
                ratio = 0.0;
            }
            kernel.push_back(UncertaintyKernelCell{dx, dy, ratio});
        }
    }

    auto inserted = uncertainty_kernel_cache_.emplace(key, std::move(kernel));
    return inserted.first->second;
}

int FM2_Planner::applyUncertaintyKernelAt(
    const int cx,
    const int cy,
    const std::vector<UncertaintyKernelCell>& kernel,
    const double vmax_ref,
    std::vector<double>& velocity_map) const {
    int changed = 0;
    for (const auto& cell : kernel) {
        const int nx = cx + cell.dx;
        const int ny = cy + cell.dy;
        if (nx < 0 || ny < 0 || nx >= nx_ || ny >= ny_) {
            continue;
        }
        const int nidx = ny * nx_ + nx;
        const double cap = vmax_ref * cell.ratio;
        if (cap < velocity_map[nidx]) {
            velocity_map[nidx] = cap;
            ++changed;
        }
    }
    return changed;
}

bool FM2_Planner::ensureBaseVelocityMap(const std::vector<int>& init_point, const int goal_idx) {
    if (base_velocity_ready_ && base_velocity_map_.size() == static_cast<std::size_t>(ns_)) {
        return true;
    }

    if (!recoverGrid()) {
        ROS_ERROR("FM2: recoverGrid failed before base velocity compute");
        return false;
    }

    FastMarching2<nDGridMap<FMCell, 2>> fm2;
    fm2.setVelocityScale(v_max_);
    fm2.setVelocityProfile(velocity_alpha_, velocity_dmax_);
    fm2.setRobotRadius(robot_radius_);
    fm2.setVelocityMode(velocity_mode_);
    fm2.setVelocitySigmoid(velocity_sigmoid_k_, velocity_sigmoid_b_);
    fm2.setEnvironment(&grid_);
    fm2.setInitialAndGoalPoints(init_point, fm2_sources_, goal_idx);

    const double saturate_distance = (velocity_dmax_ > 0.0) ? velocity_dmax_ : -1.0;
    fm2.computeFM2_velocity(saturate_distance);
    base_velocity_map_ = fm2.getVelocityMap();
    base_velocity_ready_ = (base_velocity_map_.size() == static_cast<std::size_t>(ns_));

    if (!recoverGrid()) {
        ROS_ERROR("FM2: recoverGrid failed after base velocity compute");
        return false;
    }

    if (!base_velocity_ready_) {
        ROS_ERROR("FM2: base velocity map compute failed");
        return false;
    }

    if (debug_log_) {
        ROS_INFO("FM2: base velocity map initialized (%zu cells)", base_velocity_map_.size());
    }
    return true;
}

bool FM2_Planner::applyDynamicObstacleUncertainty(
    const costmap_2d::Costmap2D* costmap,
    std::vector<double>& velocity_map) {
    if (!use_dynamic_obstacle_uncertainty_) {
        return false;
    }
    if (velocity_map.size() != static_cast<std::size_t>(ns_)) {
        return false;
    }

    const double hard_radius = std::max(0.0, dynamic_obstacle_inflation_radius_);
    const double soft_radius = std::max(hard_radius, dynamic_obstacle_uncertainty_radius_);
    const int hard_cells = std::max(
        0,
        static_cast<int>(std::ceil(hard_radius / std::max(1e-6, resolution_))));
    const int soft_cells = std::max(
        hard_cells,
        static_cast<int>(std::ceil(soft_radius / std::max(1e-6, resolution_))));

    const auto& kernel = getUncertaintyKernel(hard_cells, soft_cells);
    const double vmax_ref = getVelocityReference(velocity_map);

    const int max_samples = std::max(1, dynamic_obstacle_max_samples_);
    int changed_cells = 0;

    if (!use_dynamic_obstacle_cluster_tracking_) {
        dynamic_obstacle_tracks_.clear();
        std::vector<int> occupied_indices;
        if (obstacle_layer_ != nullptr) {
            boost::unique_lock<boost::recursive_mutex> lock(*(obstacle_layer_->getMutex()));
            collectDynamicObstacleCells(obstacle_layer_->getCharMap(), occupied_indices);
        } else if (costmap != nullptr) {
            collectDynamicObstacleCells(costmap->getCharMap(), occupied_indices);
        }

        if (occupied_indices.empty()) {
            return false;
        }

        const int sample_step = std::max(
            1,
            static_cast<int>(std::ceil(
                static_cast<double>(occupied_indices.size()) / static_cast<double>(max_samples))));
        int sampled = 0;
        for (int k = 0; k < static_cast<int>(occupied_indices.size()); k += sample_step) {
            const int idx = occupied_indices[k];
            const int mx = idx % nx_;
            const int my = idx / nx_;
            changed_cells += applyUncertaintyKernelAt(mx, my, kernel, vmax_ref, velocity_map);
            ++sampled;
        }

        if (debug_log_) {
            ROS_INFO_THROTTLE(
                1.0,
                "FM2: dynamic uncertainty legacy_cells=%zu sampled=%d changed_cells=%d step=%d",
                occupied_indices.size(),
                sampled,
                changed_cells,
                sample_step);
        }
        return changed_cells > 0;
    }

    std::vector<DynamicObstacleCluster> clusters;
    if (obstacle_layer_ != nullptr) {
        boost::unique_lock<boost::recursive_mutex> lock(*(obstacle_layer_->getMutex()));
        extractDynamicObstacleClusters(obstacle_layer_->getCharMap(), clusters);
    } else if (costmap != nullptr) {
        extractDynamicObstacleClusters(costmap->getCharMap(), clusters);
    }

    const ros::Time now = ros::Time::now();
    updateDynamicObstacleTracks(clusters, now);

    if (dynamic_obstacle_tracks_.empty()) {
        return false;
    }

    int applied_tracks = 0;
    std::vector<std::size_t> track_order(dynamic_obstacle_tracks_.size());
    for (std::size_t i = 0; i < dynamic_obstacle_tracks_.size(); ++i) {
        track_order[i] = i;
    }
    std::sort(
        track_order.begin(),
        track_order.end(),
        [&](const std::size_t lhs, const std::size_t rhs) {
            return dynamic_obstacle_tracks_[lhs].cells > dynamic_obstacle_tracks_[rhs].cells;
        });

    for (const std::size_t idx : track_order) {
        if (applied_tracks >= max_samples) {
            break;
        }
        const DynamicObstacleTrack& track = dynamic_obstacle_tracks_[idx];
        if ((now - track.last_seen).toSec() > dynamic_obstacle_track_ttl_) {
            continue;
        }
        const int mx = static_cast<int>(std::lround(track.x));
        const int my = static_cast<int>(std::lround(track.y));
        if (mx < 0 || mx >= nx_ || my < 0 || my >= ny_) {
            continue;
        }
        changed_cells += applyUncertaintyKernelAt(mx, my, kernel, vmax_ref, velocity_map);
        ++applied_tracks;
    }

    if (debug_log_) {
        ROS_INFO_THROTTLE(
            1.0,
            "FM2: dynamic uncertainty clusters=%zu tracks=%zu applied=%d changed_cells=%d",
            clusters.size(),
            dynamic_obstacle_tracks_.size(),
            applied_tracks,
            changed_cells);
    }

    return changed_cells > 0;
}

bool FM2_Planner::plan(const Node& start, const Node& goal, std::vector<Node>& path , std::vector<Node>& expand) {
    path.clear();
    expand.clear();

    std::vector<int> init_point = {grid2Index(start.x(), start.y())};
    const int goal_idx = grid2Index(goal.x(), goal.y());

    if (!recoverGrid()) {
        ROS_ERROR("FM2: recoverGrid failed");
        return false;
    }
    if (!ensureBaseVelocityMap(init_point, goal_idx)) {
        return false;
    }

    std::vector<double> runtime_velocity_map = base_velocity_map_;
    applyDynamicObstacleUncertainty(nullptr, runtime_velocity_map);

    FastMarching2<nDGridMap<FMCell, 2>> fm2;
    fm2.setVelocityScale(v_max_);
    fm2.setVelocityProfile(velocity_alpha_, velocity_dmax_);
    fm2.setRobotRadius(robot_radius_);
    fm2.setVelocityMode(velocity_mode_);
    fm2.setVelocitySigmoid(velocity_sigmoid_k_, velocity_sigmoid_b_);
    fm2.setEnvironment(&grid_);
    fm2.setInitialAndGoalPoints(init_point, fm2_sources_, goal_idx);

    const double saturate_distance = (velocity_dmax_ > 0.0) ? velocity_dmax_ : -1.0;
    if (use_gather_style_) {
        fm2.computeFM2_gather_v(init_point, runtime_velocity_map, saturate_distance);
    } else {
        fm2.computeFM2_v(runtime_velocity_map, saturate_distance);
    }

    if (debug_dump_velocity_) {
        dumpVelocityMapCsv(debug_velocity_path_);
        if (debug_log_) {
            ROS_INFO("FM2: velocity map dumped to %s", debug_velocity_path_.c_str());
        }
    }
    if (debug_visualize_) {
        const std::string title =
            "FM2 uncertainty velocity+time " + ros::this_node::getNamespace();
        plotUncertaintyVelocityAndTimeWindow(runtime_velocity_map, grid_, title, true);
    }

    typedef std::vector<std::array<double, 2>> Path;
    Path fm2_path;
    std::vector<double> path_velocity;
    bool success = false;
    if (use_gather_style_) {
        success = fm2.computePath(&fm2_path, &path_velocity, goal_idx);
    } else {
        success = fm2.computePath(&fm2_path, &path_velocity);
    }

    if (!success) {
        ROS_ERROR("FM2: computePath failed");
        recoverGrid();
        return false;
    }

    for (size_t i = 0; i < fm2_path.size(); ++i) {
        Node node;
        node.set_x(static_cast<int>(fm2_path[i][0]));
        node.set_y(static_cast<int>(fm2_path[i][1]));
        node.set_h(static_cast<double>(path_velocity[i]));
        if (debug_log_) {
            ROS_INFO("node.x():%d node.y():%d node.h():%f", node.x(), node.y(), node.h());
        }
        path.push_back(node);
    }

    if (!recoverGrid()) {
        ROS_ERROR("FM2: recoverGrid failed after planning");
        return false;
    }

    return !path.empty();
}

// 实际使用的 plan 函数
bool FM2_Planner::plan(costmap_2d::Costmap2D* costmap, const Node& start, const Node& goal, std::vector<Node>& path, std::vector<Node>& expand)
{
    path.clear();
    expand.clear();

    std::vector<int> init_point = {grid2Index(start.x(), start.y())};
    const int goal_idx = grid2Index(goal.x(), goal.y());

    if (force_full_occupancy_refresh_ && costmap != nullptr) {
        if (!updateGrid(costmap)) {
            ROS_ERROR("FM2: full occupancy refresh failed");
            return false;
        }
    }

    if (!recoverGrid()) {
        ROS_ERROR("FM2: recoverGrid failed");
        return false;
    }
    if (!ensureBaseVelocityMap(init_point, goal_idx)) {
        return false;
    }

    std::vector<double> runtime_velocity_map = base_velocity_map_;
    applyDynamicObstacleUncertainty(costmap, runtime_velocity_map);

    FastMarching2<nDGridMap<FMCell, 2>> fm2;
    fm2.setVelocityScale(v_max_);
    fm2.setVelocityProfile(velocity_alpha_, velocity_dmax_);
    fm2.setRobotRadius(robot_radius_);
    fm2.setVelocityMode(velocity_mode_);
    fm2.setVelocitySigmoid(velocity_sigmoid_k_, velocity_sigmoid_b_);
    fm2.setEnvironment(&grid_);
    fm2.setInitialAndGoalPoints(init_point, fm2_sources_, goal_idx);

    const double saturate_distance = (velocity_dmax_ > 0.0) ? velocity_dmax_ : -1.0;
    if (use_gather_style_) {
        fm2.computeFM2_gather_v(init_point, runtime_velocity_map, saturate_distance);
    } else {
        fm2.computeFM2_v(runtime_velocity_map, saturate_distance);
    }

    if (debug_dump_velocity_) {
        dumpVelocityMapCsv(debug_velocity_path_);
        if (debug_log_) {
            ROS_INFO("FM2: velocity map dumped to %s", debug_velocity_path_.c_str());
        }
    }
    if (debug_visualize_) {
        const std::string title =
            "FM2 uncertainty velocity+time " + ros::this_node::getNamespace();
        plotUncertaintyVelocityAndTimeWindow(runtime_velocity_map, grid_, title, true);
    }

    typedef std::vector<std::array<double, 2>> Path;
    Path fm2_path;
    std::vector<double> path_velocity;
    bool success = false;
    if (use_gather_style_) {
        success = fm2.computePath(&fm2_path, &path_velocity, goal_idx);
    } else {
        success = fm2.computePath(&fm2_path, &path_velocity);
    }

    if (!success) {
        ROS_ERROR("FM2: computePath failed");
        recoverGrid();
        return false;
    }

    for (size_t i = 0; i < fm2_path.size(); ++i) {
        Node node;
        node.set_x(static_cast<int>(fm2_path[i][0]));
        node.set_y(static_cast<int>(fm2_path[i][1]));
        if (i == fm2_path.size() - 1) {
            node.set_h(0.0);
        } else {
            node.set_h(static_cast<double>(path_velocity[i]));
        }
        path.push_back(node);
    }

    if (!recoverGrid()) {
        ROS_ERROR("FM2: recoverGrid failed after planning");
        return false;
    }

    return !path.empty();
}


}  // namespace global_planner

//测试绘图函数
static void plotUncertaintyVelocityAndTimeWindow(
    const std::vector<double>& velocity_map,
    nDGridMap<FMCell, 2>& grid,
    const std::string& window_title,
    bool flipY)
{
        const std::array<int,2> d = grid.getDimSizes();
        const int w = d[0];
        const int h = d[1];
        if (w <= 0 || h <= 0) {
            return;
        }
        if (velocity_map.size() != static_cast<std::size_t>(w * h)) {
            return;
        }

        double vmax = 0.0;
        for (const double v : velocity_map) {
            if (std::isfinite(v) && v > vmax) {
                vmax = v;
            }
        }
        if (vmax <= 1e-9) {
            vmax = 1.0;
        }

        double tmax = 0.0;
        for (int i = 0; i < grid.size(); ++i) {
            const double t = grid.getCell(i).getValue();
            if (std::isfinite(t) && t > tmax) {
                tmax = t;
            }
        }
        if (tmax <= 1e-9) {
            tmax = 1.0;
        }

        cimg_library::CImg<double> vel_img(w, h, 1, 1, 0);
        cimg_library::CImg<double> time_img(w, h, 1, 1, 0);
        auto map_idx = [w, h, flipY](const int x, const int y) {
            return flipY ? (w * (h - y - 1) + x) : (w * y + x);
        };

        cimg_forXY(vel_img, x, y) {
            const int idx = map_idx(x, y);
            double v = velocity_map[idx];
            if (!std::isfinite(v) || v < 0.0) {
                v = 0.0;
            }
            vel_img(x, y) = std::min(255.0, std::max(0.0, (v / vmax) * 255.0));

            const double t = grid.getCell(idx).getValue();
            if (!std::isfinite(t) || t < 0.0) {
                time_img(x, y) = 0.0;
            } else {
                time_img(x, y) = std::min(255.0, std::max(0.0, (t / tmax) * 255.0));
            }
        }

        vel_img.map(CImg<float>::jet_LUT256());
        time_img.map(CImg<float>::jet_LUT256());
        vel_img.append(time_img, 'x');
        static std::map<std::string, cimg_library::CImgDisplay> displays;
        cimg_library::CImgDisplay& disp = displays[window_title];
        vel_img.display(disp);
        disp.set_title("%s", window_title.c_str());
        disp.wait(1);
    }

static void plotVelocityMap(nDGridMap<FMCell, 2> & grid, const bool flipY)
{
        std::array<int,2> d = grid.getDimSizes();
        double max_val = 1;
        cimg_library::CImg<double> img(d[0],d[1],1,1,0);

        if (flipY)
            // Filling the image flipping Y dim. We want now top left to be the (0,0).
            cimg_forXY(img,x,y) { img(x,y) = grid[img.width()*(img.height()-y-1)+x].getVelocity()/max_val*255; }
        else
            cimg_forXY(img,x,y) { img(x,y) = grid[img.width()*y+x].getVelocity()/max_val*255; }

        img.map( CImg<float>::jet_LUT256() );
        img.display("Velocity map values", false);

    }
static void plotFrozenMap(nDGridMap<FMCell, 2> & grid, const bool flipY) {
            // TODO: image checking: B/W, correct reading, etc.
            std::array<int,2> d = grid.getDimSizes();
            cimg_library::CImg<bool> img(d[0],d[1],1,1,0);
            if (flipY)
                // Filling the image flipping Y dim. We want now top left to be the (0,0).
                cimg_forXY(img,x,y) { img(x,y) = grid.getCell(img.width()*(img.height()-y-1)+x).getState() == FMState::FROZEN; }
            else
                cimg_forXY(img,x,y) { img(x,y) = grid.getCell(img.width()*y+x).getState() == FMState::FROZEN; }

            img.display("Frozen map", false);
        }
static void plotOpenMap(nDGridMap<FMCell, 2> & grid, const bool flipY) {
            // TODO: image checking: B/W, correct reading, etc.
            std::array<int,2> d = grid.getDimSizes();
            cimg_library::CImg<bool> img(d[0],d[1],1,1,0);
            if (flipY)
                // Filling the image flipping Y dim. We want now top left to be the (0,0).
                cimg_forXY(img,x,y) { img(x,y) = grid.getCell(img.width()*(img.height()-y-1)+x).getState() == FMState::OPEN; }
            else
                cimg_forXY(img,x,y) { img(x,y) = grid.getCell(img.width()*y+x).getState() == FMState::OPEN; }

            img.display("Open map", false);
        }
static void plotOccMap(nDGridMap<FMCell, 2> & grid, const bool flipY) {
            // TODO: image checking: B/W, correct reading, etc.
            std::array<int,2> d = grid.getDimSizes();
            cimg_library::CImg<bool> img(d[0],d[1],1,1,0);
            if (flipY)
                // Filling the image flipping Y dim. We want now top left to be the (0,0).
                cimg_forXY(img,x,y) { img(x,y) = grid.getCell(img.width()*(img.height()-y-1)+x).isOccupied(); }
            else
                cimg_forXY(img,x,y) { img(x,y) = grid.getCell(img.width()*y+x).isOccupied();}

            img.display("Occupied map", false);
        }
