/*

FM2_Planner.cpp

Author: YXW

Date: 2024/11/13

Description: FM2 路径规划器

*/

#include "fm2_planner.h"
#define LETHAL 250
namespace global_planner {

FM2_Planner::FM2_Planner(costmap_2d::Costmap2D* costmap) : GlobalPlanner(costmap) {
    initializeFastMarching(costmap);
}

FM2_Planner::FM2_Planner(costmap_2d::Costmap2DROS* costmap_ros, costmap_2d::Costmap2D* costmap) : GlobalPlanner(costmap) {
    initializeFastMarchingROS(costmap_ros, costmap);
}


void FM2_Planner::initializeFastMarching(costmap_2d::Costmap2D* costmap) {
    // 设置地图尺寸
    ndims_ = 2;
    nx_= costmap->getSizeInCellsX();
    ny_ = costmap->getSizeInCellsY();
    ns_ = nx_ * ny_;
    resolution_ = costmap->getResolution();
    global_costmap_ = costmap->getCharMap();
    lethal_cost_ = LETHAL;
//for test
    // unsigned int mx,my;
    // double wx,wy;

    // 初始化地图
    dimsize_ = {nx_, ny_};
    grid_.resize(dimsize_);

    // 设置地图分辨率
    
    grid_.setLeafSize(resolution_);
    
    // 初始化占用状态（假设使用成本地图来设置占用）
    for (int i = 0; i < ns_; ++i) {
        bool is_occupied = global_costmap_[i] >= lethal_cost_;
        grid_.getCell(i).setOccupancy(!is_occupied);//False表示占用，True表示空闲
        if (is_occupied) {
            fm2_sources_.push_back(i);  // 障碍物区域作为源点
        }
    }
    GridPlotter::plotMap(grid_);  // 绘制地图
}

void FM2_Planner::initializeFastMarchingROS(costmap_2d::Costmap2DROS* costmap_ros, costmap_2d::Costmap2D* costmap) {
    ndims_ = 2;
    nx_= costmap->getSizeInCellsX();
    ny_ = costmap->getSizeInCellsY();
    ns_ = nx_ * ny_;
    resolution_ = costmap->getResolution();
    bool static_layer_exist = false;
    bool obstacle_layer_exist = false;
    unsigned char* obstacle_costmap;
    for (auto layer = costmap_ros->getLayeredCostmap()->getPlugins()->begin(); layer != costmap_ros->getLayeredCostmap()->getPlugins()->end(); ++layer)
    {
        boost::shared_ptr<costmap_2d::ObstacleLayer> obstacle_layer = boost::dynamic_pointer_cast<costmap_2d::ObstacleLayer>(*layer);
        if (obstacle_layer)  // 如果转换成功
        {
        obstacle_layer_exist = true;
        
        boost::unique_lock<boost::recursive_mutex> lock(*(obstacle_layer->getMutex()));
        obstacle_costmap = obstacle_layer->getCharMap();  //获取障碍图层的地图
        
        break;  // 找到后退出循环
        }
        boost::shared_ptr<costmap_2d::StaticLayer> static_layer = boost::dynamic_pointer_cast<costmap_2d::StaticLayer>(*layer);
        if (static_layer)  // 如果转换成功
        {
        static_layer_exist = true;
        boost::unique_lock<boost::recursive_mutex> lock(*(static_layer->getMutex()));
        static_costmap_ = static_layer->getCharMap();  // 获取静态图层的地图
        }
    }
    if(!obstacle_layer_exist)
    {
        ROS_ERROR("FM2: obstacle layer not found");
        return;
    }
    lethal_cost_ = LETHAL;


    // 初始化地图
    dimsize_ = {nx_, ny_};
    grid_.resize(dimsize_);

    // 设置地图分辨率
    
    grid_.setLeafSize(resolution_);
    
    // 初始化占用状态（假设使用成本地图来设置占用）
    for (int i = 0; i < ns_; ++i) {
        bool is_occupied = (static_costmap_[i] >= lethal_cost_) || (static_costmap_[i] == costmap_2d::NO_INFORMATION) || (obstacle_costmap[i] >= lethal_cost_);
        grid_.getCell(i).setOccupancy(!is_occupied);//setOccupancy(False)表示占用，True表示空闲
        if (is_occupied) {
            fm2_sources_.push_back(i);  // 障碍物区域作为源点
        }
    }
    // GridPlotter::plotMap(grid_);  // 绘制地图
}

// 重置网格 以进行新的计算
bool FM2_Planner::recoverGrid() {
    int i=0;
    // ROS_INFO("FM2: Resetting Grid...");
    for(i = 0; i < grid_.size(); ++i) 
    {
        //仅重置非障碍物的网格
        if(!grid_.getCell(i).isOccupied()) 
        {
            grid_.getCell(i).setValue(std::numeric_limits<double>::infinity());
            grid_.getCell(i).setVelocity(1);
            grid_.getCell(i).setState(FMState::OPEN);
        }
    }
    if (i == grid_.size()) {
        // ROS_INFO("FM2: Reset Grid Success!");
        return true;}
    return false;
}

bool FM2_Planner::resetGrid() {
    // ROS_INFO("FM2: Resetting Grid...");
    for(int i = 0; i < ns_; ++i) 
    {//重置所有网格，更新障碍地图
        grid_.getCell(i).setOccupancy(true);
        grid_.getCell(i).setValue(std::numeric_limits<double>::infinity());
        grid_.getCell(i).setVelocity(1);
        grid_.getCell(i).setState(FMState::OPEN);
    }
    // ROS_INFO("FM2: Reset Grid Success!");
    return true;
}

//更新grid_地图的函数
// 根据ObstacleLayer更新grid_地图，在grid_中更新Occupancy状态,作为新的FM2地图输入

bool FM2_Planner::updateGrid(costmap_2d::Costmap2D* costmap) 
{
    ROS_INFO("FM2: Updating Grid...");
    if (!resetGrid())
    {
        ROS_ERROR("FM2: resetGrid failed");
        return false;
    }
    fm2_sources_.clear();
    // global_costmap_ = costmap->getCharMap();
    // 更新障碍地图
    for (int i = 0; i < ns_; ++i) {
        bool is_occupied = costmap->getCharMap()[i] >= lethal_cost_;
        grid_.getCell(i).setOccupancy(!is_occupied);//setOccupancy(False)表示占用，True表示空闲
        if (is_occupied) {
            fm2_sources_.push_back(i);  // 障碍物区域作为源点
        }
    }
    return true;
}


bool FM2_Planner::plan(const Node& start, const Node& goal, std::vector<Node>& path , std::vector<Node>& expand) {

    path.clear();
    expand.clear();
    bool success = false;
    // 设置起点和目标点
    std::vector<int> init_point = {grid2Index(start.x(), start.y())};
    // ROS_INFO("FM2: init_point size: %d", init_point.size() );
    int goal_idx = grid2Index(goal.x(), goal.y());

    FastMarching2<nDGridMap<FMCell, 2>> fm2;
    fm2.setEnvironment(&grid_);
    // ROS_INFO("FM2: Set Environment: size:%d", grid_.size());
    fm2.setInitialAndGoalPoints(init_point, fm2_sources_, goal_idx);
    // ROS_INFO("FM2: Set Init & Goal, init:%d, goal:%d", init_point[0], goal_idx);
    grid_.showCoords(init_point[0]);

    // 计算 Fast Marching Square 网格
    // fm2.computeFM2();
    fm2.computeFM2(0.2); //0.2为调整最大距离进行速度饱和的阈值     速度计算中添加了最小阈值，防止速度过小的点可行
    ROS_INFO("FM2: FM2 Computed");

    // 提取路径
    typedef std::vector<std::array<double, 2>> Path;
    Path fm2_path;
    std::vector<double> path_velocity;
    success = fm2.computePath(&fm2_path, &path_velocity);
    // plotVelocityMap(grid_, true);
    if(!success) // 路径计算失败，清理网格重新规划
    {
        ROS_ERROR("FM2: computePath failed");
        resetGrid();
        return false;
    }

    // ROS_INFO("FM2: computePath");
    
    // 转换路径为 Node 格式
    for (int i = fm2_path.size() - 1; i >= 0; i--) { // 逆序输出路径
        Node node;
        node.set_x(static_cast<int>(fm2_path[i][0]));
        node.set_y(static_cast<int>(fm2_path[i][1]));
        // node.set_id(grid2Index(node.x(), node.y()));
        
        path.push_back(node);
    }

    if(!resetGrid())
    {
        ROS_ERROR("FM2: resetGrid failed");
        return false;
    }
    return !path.empty();
}

bool FM2_Planner::plan(costmap_2d::Costmap2D* costmap, const Node& start, const Node& goal, std::vector<Node>& path, std::vector<Node>& expand) 
{
    path.clear();
    expand.clear();
    updateGrid(costmap);
    bool success = false;
    // 设置起点和目标点
    std::vector<int> init_point = {grid2Index(start.x(), start.y())};
    // ROS_INFO("FM2: init_point size: %d", init_point.size() );
    int goal_idx = grid2Index(goal.x(), goal.y());

    FastMarching2<nDGridMap<FMCell, 2>> fm2;
    
    fm2.setEnvironment(&grid_);
    // ROS_INFO("FM2: Set Environment: size:%d", grid_.size());
    fm2.setInitialAndGoalPoints(init_point, fm2_sources_, goal_idx);

    // 计算 Fast Marching Square 网格
    // fm2.computeFM2();
    fm2.computeFM2(0.3); //0.2为调整最大距离进行速度饱和的阈值     速度计算中添加了最小阈值，防止速度过小的点可行
    ROS_INFO("FM2: FM2 Computed");

    // 提取路径W
    typedef std::vector<std::array<double, 2>> Path;
    Path fm2_path;
    std::vector<double> path_velocity;
    success = fm2.computePath(&fm2_path, &path_velocity);
    // plotVelocityMap(grid_, true);
    if(!success) // 路径计算失败，清理网格重新规划
    {
        ROS_ERROR("FM2: computePath failed");
        updateGrid(costmap);
        return false;
    }

    ROS_INFO("FM2: Path Computed");
    
    // 转换路径为 Node 格式
    for (int i = fm2_path.size() - 1; i >= 0; i--) { // 逆序输出路径
        Node node;
        node.set_x(static_cast<int>(fm2_path[i][0]));
        node.set_y(static_cast<int>(fm2_path[i][1]));
        // node.set_id(grid2Index(node.x(), node.y()));
        path.push_back(node);
    }

 
    if(!updateGrid(costmap))
    {
        ROS_ERROR("FM2: updateGrid failed");
        return false;
    }
    return !path.empty();

}
}  // namespace global_planner

//测试绘图函数
static void plotVelocityMap(nDGridMap<FMCell, 2> & grid, const bool flipY = true) 
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
static void plotFrozenMap(nDGridMap<FMCell, 2> & grid, const bool flipY = true) {
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
static void plotOpenMap(nDGridMap<FMCell, 2> & grid, const bool flipY = true) {
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
static void plotOccMap(nDGridMap<FMCell, 2> & grid, const bool flipY = true) {
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