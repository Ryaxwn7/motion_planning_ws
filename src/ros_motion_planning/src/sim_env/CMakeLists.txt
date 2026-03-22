cmake_minimum_required(VERSION 3.0.2)
project(sim_env)
find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  dynamic_reconfigure
  formation_msgs
  geometry_msgs
  roscpp
  rospy
  std_msgs
  message_generation
  controller_manager
  gazebo_ros
  nav_msgs
  rviz
  visualization_msgs
)

generate_dynamic_reconfigure_options(
  cfg/ShapeAssemblySwarm.cfg
)

catkin_package(
  CATKIN_DEPENDS
    actionlib_msgs
    dynamic_reconfigure
    formation_msgs
    geometry_msgs
    nav_msgs
    urdf
    visualization_msgs
    xacro
)
