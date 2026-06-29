# ==============================================================================
# ROS1 ORIGINAL (commented out)
# ==============================================================================
# cmake_minimum_required(VERSION 3.0.2)
# project(map_generator)
#
# add_compile_options(-std=c++14)
# # set(CMAKE_BUILD_TYPE "Debug")
# set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_CXX_FLAGS_DEBUG "-O0 -Wall -g -Wextra")
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
#
# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   std_msgs
#   geometry_msgs
#   pcl_conversions
# )
#
# find_package(PCL REQUIRED)
#
# catkin_package()
#
# include_directories(
#   ${catkin_INCLUDE_DIRS}
#   ${PCL_INCLUDE_DIRS}
# )
#
# add_executable(random_forest src/random_forest.cpp)
#
# target_link_libraries(random_forest
#   ${catkin_LIBRARIES}
#   ${PCL_LIBRARIES}
# )
#

# ==============================================================================
# ROS2 VERSION
# ==============================================================================
cmake_minimum_required(VERSION 3.8)
project(map_generator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(plan_env REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(random_forest src/random_forest.cpp)
ament_target_dependencies(random_forest rclcpp std_msgs nav_msgs sensor_msgs plan_env Eigen3)

ament_export_dependencies(rclcpp std_msgs nav_msgs sensor_msgs plan_env Eigen3)
ament_package()
