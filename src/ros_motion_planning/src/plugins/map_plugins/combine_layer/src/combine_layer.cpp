#include "combine_layer.h"
#include "pluginlib/class_list_macros.h"
#include <ros/ros.h>

PLUGINLIB_EXPORT_CLASS(costmap_2d::CombineLayer, costmap_2d::Layer)

namespace costmap_2d
{
    // CombineLayer::CombineLayer() : Layer() // 显式调用基类构造函数（假设基类有默认构造函数）
    // {
    // }
    void CombineLayer::onInitialize()
    {
        ros::NodeHandle nh("~/" + name_);
        enabled_ = true;
        current_ = true;
        // 参数配置
        nh.param("enabled", enabled_, enabled_);
        map_received_ = false;
        // 订阅/combine_map话题
        combine_map_sub_ = nh.subscribe("/combined_map", 1, &CombineLayer::combineMapCallback, this);

        // ROS_WARN("combine_layer initialized");
        // ROS_WARN_STREAM("Layer namespace: " << nh.getNamespace()); 
        // ROS_WARN_STREAM("Parameter enabled: " << enabled_);
    }



    void CombineLayer::combineMapCallback(const graph_planner::map::ConstPtr &msg)
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        current_map_ = *msg;
        // ROS_INFO("Received a new map");
        map_received_ = true;
    }

    void CombineLayer::updateBounds(
        double robot_x, double robot_y, double robot_yaw,
        double* min_x, double* min_y, double* max_x, double* max_y)
    {
        // ROS_WARN("Update bounds of combine_layer");
        if (!enabled_ || !map_received_) 
          {
            ROS_ERROR("combine_layer is not enabled or map is not received");
            return;
          }
          
        std::lock_guard<std::mutex> lock(map_mutex_);
        updateMapBounds(current_map_, min_x, min_y, max_x, max_y);
    }
      
    void CombineLayer::updateMapBounds(
    graph_planner::map& map,
    double* min_x, double* min_y, double* max_x, double* max_y)
    {
    // 计算地图物理边界

        const double map_width = map.gridSize[0] * map.resolution;
        const double map_height = map.gridSize[1] * map.resolution;
        
        const double origin_x = map.origin.position.x;
        const double origin_y = map.origin.position.y;
        
        // 扩展主代价地图的边界
        *min_x = std::min(*min_x, origin_x);
        *min_y = std::min(*min_y, origin_y);
        *max_x = std::max(*max_x, origin_x + map_width);
        *max_y = std::max(*max_y, origin_y + map_height);
    }

    void CombineLayer::updateCosts(
        costmap_2d::Costmap2D& master_grid, 
        int min_i, int min_j, int max_i, int max_j)
      {
          // ROS_WARN("Update costs of combine_layer");
          if (!enabled_ || !map_received_) 
          {
            ROS_ERROR("combine_layer is not enabled or map is not received");
            return;
          }
        
          std::lock_guard<std::mutex> lock(map_mutex_);
          
          const auto& map = current_map_;
          const int map_width = map.gridSize[0];
          const int map_height = map.gridSize[1];
          
          // 遍历地图所有网格
          for (int y = 0; y < map_height; ++y)
          {
            for (int x = 0; x < map_width; ++x)
            {
              const int index = y * map_width + x;
              if (index >= map.occupancyGrid.size()) continue;
              
              // 数据解析：true表示空闲，false表示障碍物
              const bool is_free = map.occupancyGrid[index];
              if (is_free) continue; // 跳过空闲区域

              // 计算世界坐标
              const double wx = map.origin.position.x + (x + 0.5) * map.resolution;
              const double wy = map.origin.position.y + (y + 0.5) * map.resolution;
              
              // 转换为代价地图坐标
              unsigned int mx, my;
              if (master_grid.worldToMap(wx, wy, mx, my))
              {
                  // 设置障碍物代价值（LETHAL_OBSTACLE=254）
                // ROS_INFO("Setting costmap obstacle at (%d, %d)", mx, my);
                master_grid.setCost(mx, my, costmap_2d::LETHAL_OBSTACLE);
              }
              else
              {
                // ROS_WARN("Failed to set obstacle at (%f, %f)", wx, wy);
              }
            }
          }
      }
}//namespace costmap_2d
