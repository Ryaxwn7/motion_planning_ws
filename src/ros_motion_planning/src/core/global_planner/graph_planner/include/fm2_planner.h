// fm2_planner.h
#ifndef FM2_PLANNER_H
#define FM2_PLANNER_H

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/obstacle_layer.h>
#include <costmap_2d/static_layer.h>
#include <dynamic_reconfigure/server.h>
#include <array>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "global_planner.h"  // 假设 AStar 也是继承于这个基类
#include "graph_planner/map.h"
#include "graph_planner/pathFM.h"
#include "graph_planner/dims.h"
#include "graph_planner/InitAndGoal.h"
#include "fastmarching/fm2/fastmarching2.hpp"
#include "fastmarching/ndgridmap/ndgridmap.hpp"
#include "fastmarching/io/gridplotter.hpp"
#include "graph_planner/FM2PlannerConfig.h"



namespace global_planner {

class FM2_Planner : public GlobalPlanner 
{
public:

    FM2_Planner(costmap_2d::Costmap2D* costmap);
    FM2_Planner(costmap_2d::Costmap2DROS* costmapROS, costmap_2d::Costmap2D* costmap);
    FM2_Planner(costmap_2d::Costmap2DROS* costmapROS, costmap_2d::Costmap2D* costmap,
                const ros::NodeHandle& nh);
    ~FM2_Planner()=default;
    bool plan(const Node& start, const Node& goal, std::vector<Node>& path, std::vector<Node>& expand) override;
    bool plan(costmap_2d::Costmap2D* costmap, const Node& start, const Node& goal, std::vector<Node>& path, std::vector<Node>& expand);

private:
    struct UncertaintyKernelCell {
        int dx;
        int dy;
        double ratio;
    };

    struct DynamicObstacleCluster {
        double x = 0.0;
        double y = 0.0;
        int cells = 0;
    };

    struct DynamicObstacleTrack {
        double x = 0.0;
        double y = 0.0;
        int cells = 0;
        ros::Time last_seen;
    };

    void initializeFastMarching(costmap_2d::Costmap2D* costmap);
    void initializeFastMarchingROS(costmap_2d::Costmap2DROS* costmapROS, costmap_2d::Costmap2D* costmap);
    void loadParams(const ros::NodeHandle& nh);
    void setupDynamicReconfigure(const ros::NodeHandle& nh);
    void reconfigureCB(graph_planner::FM2PlannerConfig& config, uint32_t level);
    void dumpVelocityMapCsv(const std::string& path);
    bool recoverGrid();//重置非障碍物grid
    bool resetGrid();//重置所有grid
    bool updateGrid(costmap_2d::Costmap2D* costmap);
    bool ensureBaseVelocityMap(const std::vector<int>& init_point, int goal_idx);
    bool applyDynamicObstacleUncertainty(const costmap_2d::Costmap2D* costmap, std::vector<double>& velocity_map);
    bool isDynamicObstacleCost(unsigned char cost) const;
    void collectDynamicObstacleCells(const unsigned char* map_data, std::vector<int>& occupied_indices) const;
    void extractDynamicObstacleClusters(
        const unsigned char* map_data,
        std::vector<DynamicObstacleCluster>& clusters) const;
    void updateDynamicObstacleTracks(
        const std::vector<DynamicObstacleCluster>& clusters,
        const ros::Time& stamp);
    double getVelocityReference(const std::vector<double>& velocity_map) const;
    const std::vector<UncertaintyKernelCell>& getUncertaintyKernel(int hard_cells, int soft_cells);
    int applyUncertaintyKernelAt(
        int cx,
        int cy,
        const std::vector<UncertaintyKernelCell>& kernel,
        double vmax_ref,
        std::vector<double>& velocity_map) const;
    void invalidateVelocityCache();
    void rebuildStaticObstacleProximityMask();
    bool isNearStaticObstacle(int idx) const;

    int ndims_;
    int nx_, ny_, ns_;
    int lethal_cost_;

    double resolution_;
    bool initialized_ = false;

    unsigned char* static_costmap_ = nullptr;
    unsigned char* obstacle_costmap_ = nullptr;
    unsigned char* global_costmap_ = nullptr;
    costmap_2d::ObstacleLayer* obstacle_layer_ = nullptr;
    nDGridMap<FMCell, 2> grid_;
    std::vector<int> fm2_sources_;
    std::array<int, 2> dimsize_;
    std::vector<double> base_velocity_map_;
    bool base_velocity_ready_ = false;
    std::map<long long, std::vector<UncertaintyKernelCell>> uncertainty_kernel_cache_;
    double uncertainty_kernel_resolution_ = -1.0;
    int static_obstacle_noise_reject_cells_ = 4;
    std::vector<uint8_t> static_obstacle_proximity_mask_;
    std::vector<DynamicObstacleTrack> dynamic_obstacle_tracks_;

    // FM2 速度场参数（与 fm2_gather 对齐）
    double v_max_ = 1.0;
    double velocity_alpha_ = 0.2;
    double velocity_dmax_ = -1.0;
    double robot_radius_ = 0.0;
    int velocity_mode_ = 0; // 0=线性, 1=Sigmoid
    double velocity_sigmoid_k_ = 0.15;
    double velocity_sigmoid_b_ = 0.0;
    bool use_gather_style_ = true;
    bool use_dynamic_obstacle_uncertainty_ = true;
    bool use_dynamic_obstacle_cluster_tracking_ = true;
    bool force_full_occupancy_refresh_ = false;
    int dynamic_obstacle_threshold_ = 253;
    bool dynamic_obstacle_unknown_is_obstacle_ = false;
    double dynamic_obstacle_inflation_radius_ = 0.2;
    double dynamic_obstacle_uncertainty_radius_ = 0.6;
    int dynamic_obstacle_stride_ = 1;
    int dynamic_obstacle_cluster_min_cells_ = 2;
    double dynamic_obstacle_track_match_radius_ = 0.5;
    double dynamic_obstacle_track_ttl_ = 0.6;
    int dynamic_obstacle_max_samples_ = 800;

    bool debug_dump_velocity_ = false;
    bool debug_visualize_ = false;
    bool debug_log_ = false;
    std::string debug_velocity_path_ = "/tmp/fm2_velocity.csv";

    dynamic_reconfigure::Server<graph_planner::FM2PlannerConfig>* dyn_server_ = nullptr;
    dynamic_reconfigure::Server<graph_planner::FM2PlannerConfig>::CallbackType dyn_cb_;
    graph_planner::FM2PlannerConfig default_config_;
    bool setup_ = false;

};

}  // namespace global_planner

static void plotVelocityMap(nDGridMap<FMCell, 2> & grid, const bool flipY);
static void plotFrozenMap(nDGridMap<FMCell, 2> & grid, const bool flipY );
static void plotOpenMap(nDGridMap<FMCell, 2> & grid, const bool flipY );
static void plotOccMap(nDGridMap<FMCell, 2> & grid, const bool flipY );

#endif  // FM2_PLANNER_H
