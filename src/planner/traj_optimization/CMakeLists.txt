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

set(OSQP_EIGEN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../3rd/osqp-eigen)
set(OSQP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../3rd/osqp)

add_subdirectory(${OSQP_DIR} ${CMAKE_CURRENT_BINARY_DIR}/osqp-build)
add_subdirectory(${OSQP_EIGEN_DIR} ${CMAKE_CURRENT_BINARY_DIR}/osqp-eigen-build)

file(GLOB_RECURSE TRAJ_OPT_SOURCES "src/*.cc")

add_library(traj_optimization ${TRAJ_OPT_SOURCES})

target_link_libraries(traj_optimization
  OsqpEigen::OsqpEigen
)

target_include_directories(traj_optimization PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../3rd/osqp/include/public
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../3rd/osqp/build/include/public
  /usr/include/eigen3
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../3rd/osqp/include/private
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../3rd/osqp/build/codegen_src/inc/private
  ${OSQP_EIGEN_DIR}/include
)

ament_export_dependencies(Eigen3)
ament_package()
