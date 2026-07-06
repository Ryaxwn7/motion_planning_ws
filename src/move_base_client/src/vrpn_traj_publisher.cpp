#include <cmath>
#include <cerrno>
#include <cstring>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>

namespace {

double distance3D(const geometry_msgs::Point& a, const geometry_msgs::Point& b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

std::string joinTopic(const std::string& prefix, int robot_id, const std::string& suffix) {
  std::string topic = prefix;
  if (!topic.empty() && topic.back() != '/') {
    topic += "/";
  }
  topic += "vrobot" + std::to_string(robot_id);
  if (!suffix.empty() && suffix.front() != '/') {
    topic += "/";
  }
  topic += suffix;
  return topic;
}

bool getRobotIds(ros::NodeHandle& nh, std::vector<int>& robot_ids) {
  if (nh.getParam("robot_ids", robot_ids) && !robot_ids.empty()) {
    return true;
  }

  int num_robots = 0;
  if (nh.getParam("num_robots", num_robots) && num_robots > 0) {
    robot_ids.reserve(num_robots);
    for (int robot_id = 1; robot_id <= num_robots; ++robot_id) {
      robot_ids.push_back(robot_id);
    }
    return true;
  }

  robot_ids = {3, 4, 5, 6};
  ROS_WARN("No robot_ids or num_robots param found, default to [3, 4, 5, 6].");
  return true;
}

bool ensureDirectory(const std::string& path, std::string& error) {
  struct stat info;
  if (stat(path.c_str(), &info) == 0) {
    if (S_ISDIR(info.st_mode)) {
      return true;
    }
    error = path + " exists but is not a directory";
    return false;
  }

  if (mkdir(path.c_str(), 0755) == 0) {
    return true;
  }

  error = "failed to create " + path + ": " + std::strerror(errno);
  return false;
}

}  // namespace

class VrpnTrajPublisher {
 public:
  VrpnTrajPublisher(ros::NodeHandle& nh, int robot_id, const std::string& input_topic,
                    const std::string& output_topic, const std::string& fixed_frame, bool use_msg_frame,
                    double min_distance, int max_poses)
      : robot_id_(robot_id),
        fixed_frame_(fixed_frame),
        use_msg_frame_(use_msg_frame),
        min_distance_(min_distance),
        max_poses_(max_poses) {
    path_.header.frame_id = use_msg_frame_ ? "" : fixed_frame_;
    pose_sub_ = nh.subscribe(input_topic, 100, &VrpnTrajPublisher::poseCallback, this);
    traj_pub_ = nh.advertise<nav_msgs::Path>(output_topic, 1, true);

    ROS_INFO_STREAM("vrobot" << robot_id_ << ": subscribe " << input_topic << ", publish " << output_topic);
  }

  int robotId() const {
    return robot_id_;
  }

  std::size_t poseCount() const {
    return path_.poses.size();
  }

  double totalLength() const {
    double length = 0.0;
    for (std::size_t i = 1; i < path_.poses.size(); ++i) {
      length += distance3D(path_.poses[i - 1].pose.position, path_.poses[i].pose.position);
    }
    return length;
  }

  bool saveCsv(const std::string& save_dir, std::string& filename, std::string& error) const {
    filename = save_dir + "/vrobot" + std::to_string(robot_id_) + "_traj.csv";
    std::ofstream file(filename);
    if (!file.is_open()) {
      error = "failed to open " + filename;
      return false;
    }

    file << "stamp,frame_id,x,y,z,qx,qy,qz,qw\n";
    for (const auto& pose : path_.poses) {
      file << pose.header.stamp.toSec() << "," << pose.header.frame_id << "," << pose.pose.position.x << ","
           << pose.pose.position.y << "," << pose.pose.position.z << "," << pose.pose.orientation.x << ","
           << pose.pose.orientation.y << "," << pose.pose.orientation.z << "," << pose.pose.orientation.w << "\n";
    }

    return true;
  }

 private:
  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (!std::isfinite(msg->pose.position.x) || !std::isfinite(msg->pose.position.y) ||
        !std::isfinite(msg->pose.position.z)) {
      ROS_WARN_THROTTLE(2.0, "vrobot%d: skip pose with non-finite position.", robot_id_);
      return;
    }

    if (!path_.poses.empty() && min_distance_ > 0.0 &&
        distance3D(path_.poses.back().pose.position, msg->pose.position) < min_distance_) {
      return;
    }

    geometry_msgs::PoseStamped pose = *msg;
    if (!use_msg_frame_) {
      pose.header.frame_id = fixed_frame_;
    } else if (path_.header.frame_id.empty()) {
      path_.header.frame_id = pose.header.frame_id;
    }

    path_.poses.push_back(pose);
    if (max_poses_ > 0 && path_.poses.size() > static_cast<std::size_t>(max_poses_)) {
      path_.poses.erase(path_.poses.begin(), path_.poses.begin() + (path_.poses.size() - max_poses_));
    }

    path_.header.stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    traj_pub_.publish(path_);
  }

  int robot_id_;
  std::string fixed_frame_;
  bool use_msg_frame_;
  double min_distance_;
  int max_poses_;
  nav_msgs::Path path_;
  ros::Subscriber pose_sub_;
  ros::Publisher traj_pub_;
};

class VrpnTrajNode {
 public:
  VrpnTrajNode() : nh_(), pnh_("~") {
    std::vector<int> robot_ids;
    getRobotIds(pnh_, robot_ids);

    std::string vrpn_prefix;
    std::string input_suffix;
    std::string output_suffix;
    std::string fixed_frame;
    bool use_msg_frame = true;
    double min_distance = 0.01;
    int max_poses = 0;

    pnh_.param<std::string>("vrpn_prefix", vrpn_prefix, "/vrpn_client_node");
    pnh_.param<std::string>("input_suffix", input_suffix, "pose");
    pnh_.param<std::string>("output_suffix", output_suffix, "traj");
    pnh_.param<std::string>("fixed_frame", fixed_frame, "world");
    pnh_.param<std::string>("save_dir", save_dir_, "/tmp/vrpn_traj");
    pnh_.param("use_msg_frame", use_msg_frame, true);
    pnh_.param("min_distance", min_distance, 0.01);
    pnh_.param("max_poses", max_poses, 0);

    publishers_.reserve(robot_ids.size());
    for (const int robot_id : robot_ids) {
      const std::string input_topic = joinTopic(vrpn_prefix, robot_id, input_suffix);
      const std::string output_topic = joinTopic(vrpn_prefix, robot_id, output_suffix);
      publishers_.emplace_back(new VrpnTrajPublisher(nh_, robot_id, input_topic, output_topic, fixed_frame,
                                                     use_msg_frame, min_distance, max_poses));
    }

    save_srv_ = pnh_.advertiseService("save_traj", &VrpnTrajNode::saveTrajCallback, this);
    ROS_INFO_STREAM("Advertise save service: " << ros::this_node::getName() << "/save_traj, save_dir=" << save_dir_);
  }

 private:
  bool saveTrajCallback(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
    std::string error;
    if (!ensureDirectory(save_dir_, error)) {
      res.success = false;
      res.message = error;
      ROS_ERROR_STREAM(error);
      return true;
    }

    std::ostringstream msg;
    bool all_saved = true;
    double all_length = 0.0;
    msg << "saved trajectories to " << save_dir_;

    for (const auto& publisher : publishers_) {
      std::string filename;
      const double length = publisher->totalLength();
      all_length += length;

      if (!publisher->saveCsv(save_dir_, filename, error)) {
        all_saved = false;
        msg << "\nvrobot" << publisher->robotId() << ": save failed, " << error << ", poses="
            << publisher->poseCount() << ", length=" << length;
        continue;
      }

      msg << "\nvrobot" << publisher->robotId() << ": " << filename << ", poses=" << publisher->poseCount()
          << ", length=" << length;
    }
    msg << "\ntotal_length=" << all_length;

    res.success = all_saved;
    res.message = msg.str();
    ROS_INFO_STREAM(res.message);
    return true;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std::string save_dir_;
  ros::ServiceServer save_srv_;
  std::vector<std::unique_ptr<VrpnTrajPublisher>> publishers_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "vrpn_traj_publisher");
  VrpnTrajNode node;
  ros::spin();
  return 0;
}
