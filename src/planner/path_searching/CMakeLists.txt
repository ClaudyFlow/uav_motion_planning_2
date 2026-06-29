# ==============================================================================
# ROS1 ORIGINAL (commented out)
# ==============================================================================
# cmake_minimum_required(VERSION 3.0.2)
# project(path_searching)
#
# find_package(Eigen3 REQUIRED)
#
# find_package(catkin REQUIRED COMPONENTS
#     roscpp
#     rospy
#     std_msgs
#     plan_env
#     cv_bridge
# )
#
# find_package(OpenCV REQUIRED)
# find_package(PCL 1.7 REQUIRED)
#
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
# set(CMAKE_BUILD_TYPE "RelWithDebInfo")
#
# catkin_package(
#     INCLUDE_DIRS include
#     LIBRARIES path_searching kdtree
#     CATKIN_DEPENDS plan_env
#     DEPENDS PCL
# )
#
# ## Specify additional locations of header files
# ## Your package locations should be listed before other locations
# include_directories(
#     include
#     ${catkin_INCLUDE_DIRS}
#     ${EIGEN3_INCLUDE_DIR}
#     ${PCL_INCLUDE_DIRS}
#     ${OpenCV_INCLUDE_DIRS}
# )
#
# link_directories(${PCL_LIBRARY_DIRS})
#
# file(GLOB_RECURSE KDTREE_SOURCES
#     "src/kdtree/*.cc"
# )
#
# file(GLOB_RECURSE PATH_SEARCHING_SOURCES
#     "src/path_searching/*.cc"
# )
#
# add_library(kdtree ${KDTREE_SOURCES})
#
# add_library(path_searching ${PATH_SEARCHING_SOURCES})
#
# target_link_libraries(path_searching
#     ${catkin_LIBRARYS}
#     kdtree
# )
#

# ==============================================================================
# ROS2 VERSION
# ==============================================================================
cmake_minimum_required(VERSION 3.8)
project(path_searching)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(plan_env REQUIRED)

add_library(kdtree ${KDTREE_SOURCES})
add_library(path_searching ${PATH_SEARCHING_SOURCES})

ament_export_dependencies(Eigen3 plan_env)
ament_package()
