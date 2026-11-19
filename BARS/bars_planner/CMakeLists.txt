cmake_minimum_required(VERSION 3.0.2)
project(bars_planner)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  quadrotor_msgs
  pcl_conversions
  pcl_ros
)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs geometry_msgs nav_msgs
                 quadrotor_msgs pcl_conversions pcl_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(bars_planner src/bars_planner.cpp)
target_link_libraries(bars_planner ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(command_bridge src/command_bridge.cpp)
target_link_libraries(command_bridge ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS
  scripts/dynamic_object_segmentation.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS bars_planner command_bridge
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
