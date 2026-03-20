#ifndef COMBINE_LAYER_H
#define COMBINE_LAYER_H

#include <ros/ros.h>
#include "graph_planner/map.h"
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>

namespace costmap_2d
{
class CombineLayer : public Layer
{
public:
  CombineLayer() = default;
  virtual ~CombineLayer() = default;

  void onInitialize() override;
  void updateBounds(double robot_x, double robot_y, double robot_yaw, 
                            double* min_x, double* min_y, double* max_x, double* max_y) override;
  void updateCosts(Costmap2D& master_grid, 
                           int min_i, int min_j, int max_i, int max_j) override;
  bool isEnabled() const { return enabled_; }

private:
  void combineMapCallback(const graph_planner::map::ConstPtr &msg);
  void updateMapBounds(graph_planner::map &msg, 
                      double* min_x, double* min_y, double* max_x, double* max_y);

  ros::Subscriber combine_map_sub_;
  graph_planner::map current_map_;
  std::mutex map_mutex_;
  bool map_received_;

};
} // namespace

#endif // COMBINE_LAYER_H