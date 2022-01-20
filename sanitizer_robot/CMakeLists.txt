cmake_minimum_required(VERSION 3.0.2)
project(sanitizer_robot)

find_package(catkin REQUIRED COMPONENTS
  OpenCV
  actionlib
  move_base_msgs
  roscpp
  std_msgs
  image_transport
  cv_bridge
  
  grid_map_demos
  grid_map_core
  grid_map_ros
  grid_map_cv
  grid_map_filters
  grid_map_loader
  grid_map_msgs
  grid_map_octomap
  grid_map_rviz_plugin
  grid_map_visualization
  geometry_msgs
  sensor_msgs
  octomap_msgs
  filters
)

catkin_package(
#  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(planner src/planner.cpp)
target_link_libraries(planner ${catkin_LIBRARIES})

add_executable(uv_visualization src/visualization/uv_grid_map.cpp)
target_link_libraries(uv_visualization ${catkin_LIBRARIES})

# Mark executables and/or libraries for installation
install(
  TARGETS 
    planner
    uv_visualization
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark other files for installation
install(
  DIRECTORY config launch 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


