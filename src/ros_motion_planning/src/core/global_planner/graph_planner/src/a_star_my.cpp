/**
 * *********************************************************
 *
 * @file: a_star_my.cpp
 * @brief: Contains the NORMAL A*_M (dijkstra and GBFS) planner class
 * @author: Yin Xiaowen
 * @date: 2024-9-14
 * @version: 1.2
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include "a_star_my.h"

namespace global_planner
{
/**
 * @brief Construct a new AStar object
 * @param costmap  the environment for path planning
 * @param dijkstra using diksktra implementation
 * @param gbfs     using gbfs implementation
 */
AStar_M::AStar_M(costmap_2d::Costmap2D* costmap, bool dijkstra, bool gbfs) : GlobalPlanner(costmap)
{
  // can not use both dijkstra and GBFS at the same time
  if (!(dijkstra && gbfs))
  {
    is_dijkstra_ = dijkstra;
    is_gbfs_ = gbfs;
  }
  else
  {
    is_dijkstra_ = false;
    is_gbfs_ = false;
  }
};

double AStar_M::GetCost(std::vector<Node>& last_path, const Node& n)
{
    // get the blind and constant cost of the node
    double min_dist = 1000000.0;;
    double blind_cost = 0.0;
    double const_cost = 0.0;
    double threshold_ = 5.0; 
    if (costmap_->getCharMap()[n.id()] == costmap_2d::NO_INFORMATION)
       {
        blind_cost = 5.0;
       }

    if(last_path.empty())
        {
          const_cost = 0.0;
        }
    else
    {
        for(auto p : last_path)
        {
            min_dist = std::min(min_dist, helper::dist(p, n));
        }
        if(min_dist > threshold_)
        const_cost = min_dist;
    }
   
    return (blind_cost + const_cost)/2.0;
} 

std::vector<Node>last_path_;

/**
 * @brief A* implementation
 * @param start  start node
 * @param goal   goal node
 * @param path   optimal path consists of Node
 * @param expand containing the node been search during the process
 * @return true if path found, else false
 */
bool AStar_M::plan(const Node& start, const Node& goal, std::vector<Node>& path, std::vector<Node>& expand)
{
   if(!path.empty())
  {
    last_path_ = path;
  }
  // clear vector
  path.clear();
  expand.clear();

  // open list and closed list
  std::priority_queue<Node, std::vector<Node>, Node::compare_cost> open_list;
  std::unordered_map<int, Node> closed_list;

  open_list.push(start);

  // get all possible motions
  const std::vector<Node> motions = Node::getMotion();

  // main process
  while (!open_list.empty())
  {
    // pop current node from open list
    auto current = open_list.top();
    open_list.pop();

    // current node exist in closed list, continue
    if (closed_list.count(current.id()))
      continue;

    closed_list.insert(std::make_pair(current.id(), current));
    expand.push_back(current);

    // goal found
    if (current == goal)
    {
      path = _convertClosedListToPath(closed_list, start, goal);
      last_path_=path;
      return true;
    }

    // explore neighbor of current node
    for (const auto& motion : motions)
    {
      // explore a new node
      auto node_new = current + motion;  // including current.g + motion.g
      node_new.set_id(grid2Index(node_new.x(), node_new.y()));
      node_new.set_pid(current.id());

      // node_new in closed list, continue
      if (closed_list.count(node_new.id()))
        continue;

      // next node hit the boundary or obstacle
      // prevent planning failed when the current within inflation
      if ((node_new.id() < 0) || (node_new.id() >= map_size_) ||
          (costmap_->getCharMap()[node_new.id()] >= costmap_2d::LETHAL_OBSTACLE * factor_ &&
           costmap_->getCharMap()[node_new.id()] >= costmap_->getCharMap()[current.id()]))
        continue;

      //##NEW##
      node_new.set_g(node_new.g() + GetCost(last_path_, node_new));
      //##END_NEW###
      // if using dijkstra implementation, do not consider heuristics cost
      if (!is_dijkstra_)
        node_new.set_h(helper::dist(node_new, goal));

      // if using GBFS implementation, only consider heuristics cost
      if (is_gbfs_)
        node_new.set_g(0.0);
      // else, g will be calculate through node_new = current + m

      open_list.push(node_new);
    }
  }

  return false;
}
}  // namespace global_planner
