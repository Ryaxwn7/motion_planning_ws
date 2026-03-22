cmake_minimum_required(VERSION 3.0.2)
project(formation_host)

find_package(catkin REQUIRED COMPONENTS
  formation_msgs
  geometry_msgs
  nav_msgs
  rospy
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    formation_msgs
    geometry_msgs
    nav_msgs
    rospy
    std_msgs
)

catkin_install_python(PROGRAMS
  scripts/shape_task_supervisor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
