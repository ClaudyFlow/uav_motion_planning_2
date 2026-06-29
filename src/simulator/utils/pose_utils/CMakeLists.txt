# ==============================================================================
# ROS1 ORIGINAL (commented out)
# ==============================================================================
# cmake_minimum_required(VERSION 3.8)
# project(pose_utils)
#
# add_compile_options(-std=c++14)
# # set(CMAKE_BUILD_TYPE "Debug")
# set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_CXX_FLAGS_DEBUG "-O0 -Wall -g -Wextra")
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
#
# find_package(catkin REQUIRED COMPONENTS roscpp)
#
# find_package(Armadillo REQUIRED)
#
# catkin_package(
#   INCLUDE_DIRS include
#   LIBRARIES pose_utils
#   # CATKIN_DEPENDS geometry_msgs nav_msgs
#   # DEPENDS system_lib
# )
#
# include_directories(
#   include
#   ${catkin_INCLUDE_DIRS}
#   ${ARMADILLO_INCLUDE_DIRS}
# )
#
# add_library(pose_utils
#   ${ARMADILLO_LIBRARIES}
#   src/pose_utils.cpp
# )
#

# ==============================================================================
# ROS2 VERSION
# ==============================================================================
cmake_minimum_required(VERSION 3.8)
project(pose_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Armadillo REQUIRED)

add_library(pose_utils ${ARMADILLO_LIBRARIES} src/pose_utils.cpp)

ament_export_dependencies(Eigen3 Armadillo)
ament_package()
