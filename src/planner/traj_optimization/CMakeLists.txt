# ==============================================================================
# ROS1 ORIGINAL (commented out)
# ==============================================================================
# cmake_minimum_required(VERSION 3.8)
# project(traj_optimization)
#
# find_package(Eigen3 REQUIRED)
# find_package(OsqpEigen REQUIRED)
#
# find_package(catkin REQUIRED COMPONENTS
#     roscpp
#     rospy
#     std_msgs
# )
#
# catkin_package(
#     INCLUDE_DIRS include
#     LIBRARIES traj_optimization
#     #  CATKIN_DEPENDS roscpp rospy std_msgs
#     #  DEPENDS system_lib
# )
#
# include_directories(
#     include
#     ${catkin_INCLUDE_DIRS}
#     ${EIGEN3_INCLUDE_DIR}
# )
#
# add_library(traj_optimization
#     src/minimum_control.cc)
#
# target_link_libraries(traj_optimization
#     ${catkin_LIBRARIES}
#     OsqpEigen::OsqpEigen)
#

# ==============================================================================
# ROS2 VERSION
# ==============================================================================
cmake_minimum_required(VERSION 3.8)
project(traj_optimization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

file(GLOB_RECURSE TRAJ_OPT_SOURCES "src/*.cc")

add_library(traj_optimization ${TRAJ_OPT_SOURCES})

target_include_directories(traj_optimization PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_export_dependencies(Eigen3)
ament_package()
