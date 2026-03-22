cmake_minimum_required(VERSION 3.0.2)
project(formation_robot)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  formation_msgs
  geometry_msgs
  move_base_msgs
  rospy
  tf
)

catkin_package(
  CATKIN_DEPENDS
    actionlib
    formation_msgs
    geometry_msgs
    move_base_msgs
    rospy
    tf
)

catkin_install_python(PROGRAMS
  scripts/shape_task_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
