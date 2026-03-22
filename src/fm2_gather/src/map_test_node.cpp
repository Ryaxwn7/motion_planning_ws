#include <ros/ros.h>
#include <fm2_gather/map.h>
#define cimg_use_png
#include "fastmarching/thirdparty/CImg.h"
#include <geometry_msgs/Pose.h>
#include "fastmarching/io/gridplotter.hpp"
#include "fastmarching/io/gridwriter.hpp"
using namespace cimg_library;

fm2_gather::map map_msg;


void mapCallback(const fm2_gather::map::ConstPtr& msg) {
    // 获取图像数据
    if(msg->gridSize.empty())
    {
        ROS_WARN("Grid size is empty, please check the map message");
        return;
    }
    map_msg.occupancyGrid = msg->occupancyGrid;
    int width = msg->gridSize[0];
    int height = msg->gridSize[1];
    float resolution = msg->resolution;

    // 创建CImg图像
    CImg<unsigned char> image(width, height, 1, 1, 0);
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int index = y * width + x;
            image(x, y) = map_msg.occupancyGrid[index] ? 255 : 0;
        }
    }

    // 显示图像
    // image.save_png("/home/bsrl-ubuntu/fm2_gather/combined_map/map.png");
    CImgDisplay display(image, "Map Visualization");
    while (!display.is_closed()) {
        display.wait();
    }
}

int main(int argc, char **argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "map_visualizer");
    ros::NodeHandle nh;

    // 订阅map消息
    ros::Subscriber sub = nh.subscribe<fm2_gather::map>("/combined_map", 1, mapCallback);

    // 进入ROS循环
    ros::spin();

    return 0;
}
