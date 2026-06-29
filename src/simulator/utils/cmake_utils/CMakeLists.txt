# ==============================================================================
# ROS1 ORIGINAL (commented out)
# ==============================================================================
# cmake_minimum_required(VERSION 3.8)
# project(cmake_utils)
#
# find_package(catkin REQUIRED COMPONENTS roscpp)
#
# set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)
#
# file(GLOB CMAKE_UILTS_FILES cmake/*.cmake)
#
# catkin_package(
#   CFG_EXTRAS ${CMAKE_UILTS_FILES}
# )
#

# ==============================================================================
# ROS2 VERSION
# ==============================================================================
cmake_minimum_required(VERSION 3.8)
project(cmake_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)


ament_package()
