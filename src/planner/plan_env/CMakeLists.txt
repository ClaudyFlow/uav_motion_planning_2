# ==============================================================================
# ROS1 ORIGINAL (commented out)
# ==============================================================================
# cmake_minimum_required(VERSION 3.8)
# project(plan_env)
#
# set(CMAKE_BUILD_TYPE "Release")
# ADD_COMPILE_OPTIONS(-std=c++11)
# ADD_COMPILE_OPTIONS(-std=c++14)
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
#
# find_package(OpenCV REQUIRED)
#
#
# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   rospy
#   std_msgs
#   visualization_msgs
#   cv_bridge
#   message_filters
#
# )
#
# find_package(Eigen3 REQUIRED)
# find_package(PCL 1.7 REQUIRED)
#
# catkin_package(
#   INCLUDE_DIRS include
#   LIBRARIES plan_env
#   CATKIN_DEPENDS roscpp std_msgs
#   # DEPENDS system_lib
# )
#
# include_directories(
#   SYSTEM
#   include
#   ${catkin_INCLUDE_DIRS}
#   ${EIGEN3_INCLUDE_DIR}
#   ${PCL_INCLUDE_DIRS}
#   ${OpenCV_INCLUDE_DIRS}
# )
#
# link_directories(${PCL_LIBRARY_DIRS})
#
# file(GLOB_RECURSE PLAN_ENV_SOURCES
#     "src/plan_env/*.cc"
# )
#
# add_library(plan_env ${PLAN_ENV_SOURCES})
#
# target_link_libraries(plan_env
#   ${catkin_LIBRARIES}
#   ${PCL_LIBRARIES}
# )
#

# ==============================================================================
# ROS2 VERSION
# ==============================================================================
cmake_minimum_required(VERSION 3.8)
project(plan_env)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
file(GLOB_RECURSE PLAN_ENV_SOURCES "src/plan_env/*.cc")

find_package(Eigen3 REQUIRED)

add_library(plan_env ${PLAN_ENV_SOURCES})

ament_export_dependencies(Eigen3)
ament_package()
