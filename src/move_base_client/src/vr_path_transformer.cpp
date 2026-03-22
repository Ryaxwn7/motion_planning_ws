#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <string>
#include <signal.h> // 引入信号处理头文件
#include <fstream>  // 用于保存数据到文件

std::vector<nav_msgs::Path> saved_paths;


class RobotPathPublisher {
public:
    RobotPathPublisher(int robot_id) : nh_("~"), robot_id_(robot_id) {
        // Construct topic names based on robot ID
        std::string pose_topic = "/vrpn_client_node/robot" + std::to_string(robot_id_) + "/pose_throttle";
        std::string path_topic = "/vrpn_client_node/robot" + std::to_string(robot_id_) + "/trajectory";

        pose_sub_ = nh_.subscribe(pose_topic, 10, &RobotPathPublisher::poseCallback, this);
        path_pub_ = nh_.advertise<nav_msgs::Path>(path_topic, 10);

        ROS_INFO_STREAM("Subscribed to: " << pose_topic);
        ROS_INFO_STREAM("Publishing to: " << path_topic);

        path_.header.frame_id = "map"; // Set the frame ID for the path
    }

    void save_path() const {
        std::string filename = "/home/bsrl-ubuntu/saved_paths/robot_" + std::to_string(robot_id_) + "_path.csv";
        std::ofstream file(filename);
        if (!file.is_open()) {
            ROS_ERROR_STREAM("Failed to open file: " << filename);
            return;
        }

        // 写入CSV表头
        file << "x,y,z\n";

        // 写入路径点
        for (const auto& pose : path_.poses) {
            file << pose.pose.position.x << ","
                 << pose.pose.position.y << ","
                 << pose.pose.position.z << "\n";
        }

        file.close();
        ROS_INFO_STREAM("Path saved to " << filename);
    }

    double print_path_length() const {
        double length = 0.0;
        if (path_.poses.size() < 2) {
            ROS_WARN_STREAM("Path has less than 2 points, cannot calculate length.");
            return 0;
        }

        for (size_t i = 1; i < path_.poses.size(); ++i) {
            const auto& p1 = path_.poses[i - 1].pose.position;
            const auto& p2 = path_.poses[i].pose.position;
            double dx = p2.x - p1.x;
            double dy = p2.y - p1.y;
            double dz = p2.z - p1.z;
            length += std::sqrt(dx * dx + dy * dy + dz * dz);
        }

        ROS_INFO_STREAM("Path length: " << length);
        return length;
    }


private:
    ros::NodeHandle nh_;
    ros::Subscriber pose_sub_;
    ros::Publisher path_pub_;
    nav_msgs::Path path_;
    int robot_id_;

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        // Add the received pose to the path
        path_.poses.push_back(*msg);
        path_.header.stamp = ros::Time::now(); // Update the timestamp

        // Publish the updated path
        path_pub_.publish(path_);
    }
};

std::vector<std::shared_ptr<RobotPathPublisher>> publishers;
void saveDataAndExit(int signum) {
    ROS_INFO("Saving data before exiting...");
    for(int i = 0; i < publishers.size(); ++i) {
        auto publisher = publishers[i];
        ROS_INFO_STREAM("Saving path for robot");
        publisher->save_path();
        publisher->print_path_length();
    }

    ros::shutdown(); // 关闭 ROS 节点

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_path_publisher");
    ros::NodeHandle nh("~");

    // Register signal handler for SIGINT (Ctrl+C)
    signal(SIGINT, saveDataAndExit);

    int num_robots = 4; // Default to 4 robots


    std::string paramname1, paramname2;
    std::vector<int> robot_ids={1,2,3,4};
    if(nh.searchParam("num_robots",paramname1)&& nh.searchParam("robot_ids",paramname2))
    {
        nh.getParam(paramname1  , num_robots);
        nh.getParam(paramname2  , robot_ids);
    }
    else
    {
        ROS_ERROR("Failed to get /robot_ids or /num_robots parameters");
        num_robots = 4;
        robot_ids = {3,4,5,6};
    }

    for (int i = 0; i < num_robots; ++i) {
        publishers.push_back(std::make_shared<RobotPathPublisher>(robot_ids[i]));
    }

    ros::spin();
    return 0;
}