#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_srvs/Empty.h>

class CostmapClearer
{
public:
    CostmapClearer() : nh_("~")
    {
        setlocale(LC_ALL,"");
        nh_.getParam("name_space", name_space);
        std::string initial_pose_topic = "/" + name_space + "/initialpose";
        std::string servive_name = "/" + name_space + "/move_base/clear_costmaps";
        if (name_space.empty()) {
            initial_pose_topic = "/initialpose";
            servive_name = "/move_base/clear_costmaps";
        }
        ROS_ERROR("CostmapClearer: name_space: %s, initial_pose_topic: %s, servive_name: %s", name_space.c_str(), initial_pose_topic.c_str(), servive_name.c_str());
        initial_pose_sub_ = nh_.subscribe(initial_pose_topic, 1, &CostmapClearer::initialPoseCallback, this);
        clear_costmaps_client_ = nh_.serviceClient<std_srvs::Empty>(servive_name);

    }

    void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
    {
        std_srvs::Empty srv;
        if (clear_costmaps_client_.call(srv))
        {
            ROS_INFO("成功清除代价地图！");
        }
        else
        {
            ROS_ERROR("代价地图清除失败.");
        }
    }

    std::string name_space;

private:
    ros::NodeHandle nh_;
    ros::Subscriber initial_pose_sub_;
    ros::ServiceClient clear_costmaps_client_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "costmap_cleaner");
    CostmapClearer clearer;
    ros::spin();
    return 0;
}