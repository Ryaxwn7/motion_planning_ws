// fm2_planner.h
#ifndef FM2_PLANNER_H
#define FM2_PLANNER_H

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/obstacle_layer.h>
#include <costmap_2d/static_layer.h>
#include <vector>

#include "global_planner.h"  // 假设 AStar 也是继承于这个基类
#include "graph_planner/map.h"
#include "graph_planner/pathFM.h"
#include "graph_planner/dims.h"
#include "graph_planner/InitAndGoal.h"
#include "fastmarching/fm2/fastmarching2.hpp"
#include "fastmarching/ndgridmap/ndgridmap.hpp"
#include "fastmarching/io/gridplotter.hpp"



namespace global_planner {

class FM2_Planner : public GlobalPlanner 
{
public:

    FM2_Planner(costmap_2d::Costmap2D* costmap);
    FM2_Planner(costmap_2d::Costmap2DROS* costmapROS, costmap_2d::Costmap2D* costmap);

    bool plan(const Node& start, const Node& goal, std::vector<Node>& path, std::vector<Node>& expand);
    bool plan(costmap_2d::Costmap2D* costmap, const Node& start, const Node& goal, std::vector<Node>& path, std::vector<Node>& expand);

private:
    void initializeFastMarching(costmap_2d::Costmap2D* costmap);
    void initializeFastMarchingROS(costmap_2d::Costmap2DROS* costmapROS, costmap_2d::Costmap2D* costmap);
    bool recoverGrid();//重置非障碍物grid
    bool resetGrid();//重置所有grid
    bool updateGrid(costmap_2d::Costmap2D* costmap);
    int ndims_;
    int nx_, ny_, ns_;
    int lethal_cost_;

    double resolution_;
    bool initialized_ = false;

    unsigned char* static_costmap_;
    unsigned char* obstacle_costmap_; //用于执行过程中的更新
    unsigned char* global_costmap_; //用于执行过程中的更新
    nDGridMap<FMCell, 2> grid_;
    std::vector<int> fm2_sources_;
    std::array<int, 2> dimsize_;

};

}  // namespace global_planner

static void plotVelocityMap(nDGridMap<FMCell, 2> & grid, const bool flipY);
static void plotFrozenMap(nDGridMap<FMCell, 2> & grid, const bool flipY );
static void plotOpenMap(nDGridMap<FMCell, 2> & grid, const bool flipY );
static void plotOccMap(nDGridMap<FMCell, 2> & grid, const bool flipY );

#endif  // FM2_PLANNER_H
