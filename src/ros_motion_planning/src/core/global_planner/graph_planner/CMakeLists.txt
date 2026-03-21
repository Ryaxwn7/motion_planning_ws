cmake_minimum_required(VERSION 3.0.2)
project(graph_planner)

find_package(catkin REQUIRED COMPONENTS
  angles
  roscpp
  costmap_2d
  geometry_msgs
  nav_core
  nav_msgs
  navfn
  pluginlib
  tf2_geometry_msgs
  tf2_ros
  global_planner
  voronoi_layer
  utils
  curve_generation
  dynamic_reconfigure
  genmsg
  std_msgs
)

find_package(X11 REQUIRED)

###new:genmsg###
###new###
add_message_files(
  FILES
  dims.msg
  InitAndGoal.msg
  map.msg
  pathFM.msg
  pathTurtlebot.msg
)
###new###
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

generate_dynamic_reconfigure_options(
  cfg/FM2Planner.cfg
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS global_planner voronoi_layer utils curve_generation dynamic_reconfigure
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${X11_INCLUDE_DIR}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/graph_planner.cpp
  src/fm2_planner.cpp
  src/fm2star_planner.cpp
  src/a_star_my.cpp
  src/a_star.cpp
  src/jump_point_search.cpp
  src/d_star.cpp
  src/lpa_star.cpp
  src/d_star_lite.cpp
  src/voronoi.cpp
  src/theta_star.cpp
  src/lazy_theta_star.cpp
  src/s_theta_star.cpp
  src/hybrid_a_star.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${X11_LIBRARIES}
)
