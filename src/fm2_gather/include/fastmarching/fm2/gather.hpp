#ifndef GATHER_HPP_
#define GATHER_HPP_

#include <iostream>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <fstream>
#include <array>
#include <limits>
 //#include <fast_methods/fm/fmm.hpp>
#include<vector>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include "fastmarching/ndgridmap/ndgridmap.hpp"
#include "fastmarching/fm2/fastmarching2.hpp"
#include "fm2_gather/map.h"

namespace bg = boost::geometry;


// #define M_PI       3.141592653589793238462  // pi
#define  PI  acos(-1)

typedef struct{
    int idx;
    double val;
} VelocityElement;

struct CompareElement {
    bool operator()(const VelocityElement& a, const VelocityElement& b) {
        return a.val < b.val; // 最大堆需返回较小值优先级更低
    }
};

class Gather
{
public:

   Gather(int robotNum, double hypotenuse, double resolution, double origin_x, double origin_y, int grid_size_x, int grid_size_y) : robotNum(robotNum), 
   hypotenuse(hypotenuse), resolution_(resolution), ori_x(origin_x), ori_y(origin_y), gs_x(grid_size_x), gs_y(grid_size_y)
    {}

   unsigned int getMinIdx(nDGridMap<FMCell, 2> grid)
   {
      double min=10e6;
      int min_idx=0;
       for(unsigned int j=0;j<grid.size();j++)
       {
        
             if ( grid.getCell(j).getValue() < min && grid.getCell(j).getOccupancy() )
             {
                   min=grid.getCell(j).getValue();
                   min_idx=j;
             }
       }
       return min_idx;

   }


   bool GenerateGoals(std::vector<geometry_msgs::PoseStamped> robot_poses_, 
    std::array<float,2> goal_centre_, 
    std::vector<std::array<float,2>>& goals) {
    goals.clear();
    // 生成等距目标点
    for(int i=0; i<robotNum; i++) {
    std::array<float,2> goal;
    goal[0] = goal_centre_[0] + hypotenuse * cos(2*PI/robotNum * i);
    goal[1] = goal_centre_[1] + hypotenuse * sin(2*PI/robotNum * i);
    goals.push_back(goal);
    }

    // 构建代价矩阵
    std::vector<std::vector<int>> cost(robotNum, std::vector<int>(robotNum));
    for (int i=0; i<robotNum; ++i) {
    for (int j=0; j<robotNum; ++j) {
    float dx = robot_poses_[i].pose.position.x - goals[j][0];
    float dy = robot_poses_[i].pose.position.y - goals[j][1];
    cost[i][j] = static_cast<int>(sqrt(dx*dx + dy*dy) * 1000);
    }
    }

    // 执行匈牙利算法
    std::vector<int> match;
    hungarian(cost, match);

    // 分配目标点
    std::vector<std::array<float,2>> assigned_goals(robotNum);
    for (int j=0; j<robotNum; ++j) {
    int i = match[j]; // 机器人i分配到目标j
    assigned_goals[i] = goals[j];
    }

    goals = assigned_goals;
    return true;
    }

    

    bool world2mapidx(float wx, float wy, int &idx)
    {
        if(wx < ori_x || wy < ori_y)
        {
            ROS_ERROR("World coordinates (%f, %f) are out of bounds(%f, %f)", wx, wy, ori_x, ori_y); 
            return false;
        }
        int mx = (int)round((wx - ori_x) / resolution_);
        int my = (int)round((wy - ori_y) / resolution_);
        
        if(mx < gs_x && my < gs_y && mx >= 0 && my >= 0)
        {
            idx = mx + my * gs_x;
            return true;
        }
        return false;
    }

    bool GenerateGoals_idx(std::vector<geometry_msgs::PoseStamped> robot_poses_, 
        std::array<float,2> goal_centre_, 
        std::vector<int>& goal_idxs_) {
        goal_idxs_.clear();
        // 生成等距目标点
        std::vector<std::array<float,2>> goals;
        for(int i=0; i<robotNum; i++) {
        std::array<float,2> goal;
        goal[0] = goal_centre_[0] + hypotenuse * cos(2*PI/robotNum * i);
        goal[1] = goal_centre_[1] + hypotenuse * sin(2*PI/robotNum * i);
        goals.push_back(goal);
        }
        std::cout << "Goals: " << std::endl;
        for(int i=0; i<robotNum; i++)
        {
            std::cout << "(" << goals[i][0] << ", " << goals[i][1] << ")" << std::endl;
        }
    
        // 构建代价矩阵
        std::vector<std::vector<int>> cost(robotNum, std::vector<int>(robotNum));
        for (int i=0; i<robotNum; ++i) {
        for (int j=0; j<robotNum; ++j) {
        float dx = robot_poses_[i].pose.position.x - goals[j][0];
        float dy = robot_poses_[i].pose.position.y - goals[j][1];
        cost[i][j] = static_cast<int>(sqrt(dx*dx + dy*dy) * 1000);
        }
        }
    
        // 执行匈牙利算法
        std::vector<int> match;
        hungarian(cost, match);
    
        // 分配目标点
        std::vector<std::array<float,2>> assigned_goals(robotNum);
        for (int j=0; j<robotNum; ++j) {
        int i = match[j]; // 机器人i分配到目标j
        assigned_goals[i] = goals[j];
        }
    
        goals = assigned_goals;
        for(int i=0; i<robotNum; i++)
        {
            int goal_idx;
            if(world2mapidx(assigned_goals[i][0], assigned_goals[i][1], goal_idx))
            {
                goal_idxs_.push_back(goal_idx);
                std::cout << "Goal " << i << " idx: " << goal_idx << std::endl;
            }
            else
            {
                ROS_ERROR("World coordinates (%f, %f) are out of bounds", assigned_goals[i][0], assigned_goals[i][1]); 
            }
        }
        return true;
    }

    //参数列表  
    // std::vector<geometry_msgs::PoseStamped> robot_poses_; //机器人位姿
    // std::array<float,2> goal_centre_; //中心目标点坐标
    // std::vector<std::array<float,2>>& goals; //分配的目标点坐标 
    // std::vector<std::shared_ptr<FastMarching2<nDGridMap<FMCell,2>>>> fm2_solvers_; //fm2求解器
    // std::string format; //队形 （C， L） (圆形、直线型)
    // 
    // bool GenerateGoals_check(int min_idx, std::array<float,2>goal_centre_, std::vector<std::array<float,2>>& goals, std::vector<std::shared_ptr<FastMarching2<nDGridMap<FMCell,2>>>> fm2_solvers_, std::string format, double ori_x, double ori_y, double resolution_, int gs_x, int gs_y)
    // {
    //     goals.clear();
    //     int robotNum = fm2_solvers_.size();
    //     // 生成等距目标点
    //     int goal_idx[robotNum];
    //     for(int i=0; i<robotNum; i++) {
    //         std::array<float,2> goal;
    //         if(format == "C")
    //         {
    //             goal[0] = goal_centre_[0] + hypotenuse * cos(2*PI/robotNum * i);
    //             goal[1] = goal_centre_[1] + hypotenuse * sin(2*PI/robotNum * i);
    //             Gather::world2mapidx(goal[0], goal[1], goal_idx[i], resolution_, ori_x, ori_y, gs_x, gs_y);
    //         }
    //         else if(format == "L")// TODO: 直线型队形
    //         {
    //             goal[0] = goal_centre_[0] + hypotenuse * (i - robotNum/ 2.0);
    //             goal[1] = goal_centre_[1];
    //             Gather::world2mapidx(goal[0], goal[1], goal_idx[i], resolution_, ori_x, ori_y, gs_x, gs_y);
    //         }
    //         goals.push_back(goal);
    //     }
    //     // std::vector<std::shared_ptr<Gather::PathG>> fmpaths;
    //     // std::vector<std::shared_ptr<Gather::Path_vG>> fmpaths_v;
    //     // for(int i=0; i<robotNum; i++)
    //     // {
    //     //     fmpaths.push_back(std::make_shared<Gather::PathG>());
    //     //     fmpaths_v.push_back(std::make_shared<Gather::Path_vG>());
    //     // }
    //     // 构建代价矩阵
    //     std::vector<std::vector<int>> cost(robotNum, std::vector<int>(robotNum));
    //     for (int i=0; i<robotNum; ++i) { //robot i
    //         for (int j=0; j<robotNum; ++j) { // goal j
    //             std::vector<std::array<double, 2>> path;
    //             std::vector<double> path_v;

    //             fm2_solvers_[i]->computePath(&path, &path_v, goal_idx[j]);
    //             if(path.empty())
    //             {
    //                 std::cout << "path is empty" << std::endl;
    //                 return false;
    //             }
    //             float path_len = 0;
    //             for(int k=0; k<path.size()-1; k++)
    //             {
    //                 float dx = path[k+1][0] - path[k][0];
    //                 float dy = path[k+1][1] - path[k][1];
    //                 path_len += sqrt(dx*dx + dy*dy);
    //             }
    //             cost[i][j] = static_cast<int>(path_len * 1000);
    //         }
    //     }
    //      // 执行匈牙利算法
    //     std::vector<int> match;
    //     hungarian(cost, match);

    //     // 分配目标点
    //     std::vector<std::array<float,2>> assigned_goals(robotNum);
    //     for (int j=0; j<robotNum; ++j) {
    //     int i = match[j]; // 机器人i分配到目标j
    //     assigned_goals[i] = goals[j];
    //     }

    //     goals = assigned_goals;
    //     return true;
    // }

    bool GenerateGoals_v(int min_idx, std::array<float,2>goal_centre_, std::vector<std::array<float,2>>& goals, std::vector<std::shared_ptr<FastMarching2<nDGridMap<FMCell,2>>>> fm2_solvers_, std::string format)
    {
        goals.clear();
        int robotNum = fm2_solvers_.size();
        // 生成等距目标点
        int goal_idx[robotNum];
        for(int i=0; i<robotNum; i++) {
            std::array<float,2> goal;
            if(format == "C")
            {
                goal[0] = goal_centre_[0] + hypotenuse * cos(2*PI/robotNum * i);
                goal[1] = goal_centre_[1] + hypotenuse * sin(2*PI/robotNum * i);
                Gather::world2mapidx(goal[0], goal[1], goal_idx[i]);
            }
            else if(format == "L")// TODO: 直线型队形
            {
                goal[0] = goal_centre_[0] + hypotenuse * (i - robotNum/ 2.0);
                goal[1] = goal_centre_[1];
                Gather::world2mapidx(goal[0], goal[1], goal_idx[i]);
            }
            goals.push_back(goal);
        }
        // std::vector<std::shared_ptr<Gather::PathG>> fmpaths;
        // std::vector<std::shared_ptr<Gather::Path_vG>> fmpaths_v;
        // for(int i=0; i<robotNum; i++)
        // {
        //     fmpaths.push_back(std::make_shared<Gather::PathG>());
        //     fmpaths_v.push_back(std::make_shared<Gather::Path_vG>());
        // }
        // 构建代价矩阵
        std::vector<std::vector<int>> cost(robotNum, std::vector<int>(robotNum));
        for (int i=0; i<robotNum; ++i) { //robot i
            for (int j=0; j<robotNum; ++j) { // goal j
                std::vector<std::array<double, 2>> path;
                std::vector<double> path_v;

                fm2_solvers_[i]->computePath(&path, &path_v, goal_idx[j]);
                if(path.empty())
                {
                    std::cout << "path is empty" << std::endl;
                    return false;
                }
                float path_len = 0;
                for(int k=0; k<path.size()-1; k++)
                {
                    float dx = path[k+1][0] - path[k][0];
                    float dy = path[k+1][1] - path[k][1];
                    path_len += sqrt(dx*dx + dy*dy);
                }
                cost[i][j] = static_cast<int>(path_len * 1000);
            }
        }
         // 执行匈牙利算法
        std::vector<int> match;
        hungarian(cost, match);

        // 分配目标点
        std::vector<std::array<float,2>> assigned_goals(robotNum);
        for (int j=0; j<robotNum; ++j) {
        int i = match[j]; // 机器人i分配到目标j
        assigned_goals[i] = goals[j];
        }

        goals = assigned_goals;
        return true;
    }
      
    bool GenerateGoals_p(std::vector<std::shared_ptr<std::vector<std::array<double, 2>>>> robot_paths_, 
    std::array<float,2> goal_centre_, 
    std::vector<std::array<float,2>>& goals) {
    goals.clear();
    std::vector<std::array<double, 2>> robot_last_poses;
    int range = 2* hypotenuse / resolution_;
    // 生成等距目标点
    for(int i=0; i<robotNum; i++) {
    std::array<float,2> goal;
    std::array<double, 2> robot_last_pose;
    goal[0] = goal_centre_[0] + hypotenuse * cos(2*PI/robotNum * i);
    goal[1] = goal_centre_[1] + hypotenuse * sin(2*PI/robotNum * i);
    goals.push_back(goal);
    if(robot_paths_[i]->size() > range)
        {
            robot_last_poses.push_back(robot_paths_[i]->at(range)); //path[0] 是终点
        }
    else
        {
            robot_last_poses.push_back(robot_paths_[i]->at(robot_paths_[i]->size()-1));
        }
    }

    // 构建代价矩阵
    std::vector<std::vector<int>> cost(robotNum, std::vector<int>(robotNum));
    for (int i=0; i<robotNum; ++i) {
    for (int j=0; j<robotNum; ++j) {
    float dx = robot_last_poses[i][0] - goals[j][0];
    float dy = robot_last_poses[i][1] - goals[j][1];
    cost[i][j] = static_cast<int>(sqrt(dx*dx + dy*dy) * 1000);
    }
    }

    // 执行匈牙利算法
    std::vector<int> match;
    hungarian(cost, match);

    // 分配目标点
    std::vector<std::array<float,2>> assigned_goals(robotNum);
    for (int j=0; j<robotNum; ++j) {
    int i = match[j]; // 机器人i分配到目标j
    assigned_goals[i] = goals[j];
    }

    goals = assigned_goals;
    return true;
    }


   double getHypotenus()
   {
       return hypotenuse;
   }


private:
//机器人个数
        int  robotNum;
        
//机器人斜边长度
        double hypotenuse;
        double resolution_;
        double ori_x, ori_y;
        int gs_x, gs_y;


   int hungarian(const std::vector<std::vector<int>>& cost_matrix, std::vector<int>& match) {
         int n = cost_matrix.size();
         std::vector<int> u(n+1), v(n+1), p(n+1), way(n+1);
         std::fill(u.begin(), u.end(), 0);
         std::fill(v.begin(), v.end(), 0);
         std::fill(p.begin(), p.end(), 0);
         std::fill(way.begin(), way.end(), 0);
 
         for (int i=1; i<=n; ++i) {
             p[0] = i;
             int j0 = 0;
             std::vector<int> minv (n+1, std::numeric_limits<int>::max());
             std::vector<bool> used (n+1, false);
             do {
                 used[j0] = true;
                 int i0 = p[j0], delta = std::numeric_limits<int>::max(), j1;
                 for (int j=1; j<=n; ++j) {
                     if (!used[j]) {
                         int cur = cost_matrix[i0-1][j-1] - u[i0] - v[j];
                         if (cur < minv[j]) {
                             minv[j] = cur;
                             way[j] = j0;
                         }
                         if (minv[j] < delta) {
                             delta = minv[j];
                             j1 = j;
                         }
                     }
                 }
                 for (int j=0; j<=n; ++j) {
                     if (used[j]) {
                         u[p[j]] += delta;
                         v[j] -= delta;
                     } else {
                         minv[j] -= delta;
                     }
                 }
                 j0 = j1;
             } while (p[j0] != 0);
             do {
                 int j1 = way[j0];
                 p[j0] = p[j1];
                 j0 = j1;
             } while (j0);
         }
 
         match.resize(n);
         for (int j=1; j<=n; ++j)
             match[j-1] = p[j]-1;
         return -v[0];
     }



//机器人组成图形的顶点的坐标
    //  std::vector< boost::geometry::model::d2::point_xy<double> >  graphicVertex;
    boost::shared_ptr <std::vector<nDGridMap<FMCell, 2>>>  grids_ptr;
};




#endif