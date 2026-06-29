# ==============================================================================
# ROS1 ORIGINAL (commented out)
# ==============================================================================
# cmake_minimum_required(VERSION 3.8)
# project(fake_drone)
#
# add_compile_options(-std=c++14)
# # set(CMAKE_BUILD_TYPE "Debug")
# set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_CXX_FLAGS_DEBUG "-O0 -Wall -g -Wextra")
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
#
# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   rospy
#   nav_msgs
#   quadrotor_msgs
# )
#
# find_package(Eigen3 REQUIRED)
#
# catkin_package()
#
# include_directories(
#   # include
#   ${catkin_INCLUDE_DIRS}
#   ${EIGEN3_INCLUDE_DIR}
# )
#
# add_executable(fake_drone src/fake_drone.cc)
#
# target_link_libraries(fake_drone
#   ${catkin_LIBRARIES}
# )


# ==============================================================================
# ROS2 VERSION
# ==============================================================================

cmake_minimum_required(VERSION 3.8)
project(fake_drone)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
include(CMakeParseArguments)
function(ament_target_dependencies TARGET)
    cmake_parse_arguments(ATD "" "" "DEPENDENCIES" ${ARGN})
    if(ATD_DEPENDENCIES)
        foreach(DEP ${ATD_DEPENDENCIES})
            find_package(${DEP} REQUIRED)
            if(DEFINED ${DEP}_INCLUDE_DIRS})
                target_include_directories(${TARGET} PUBLIC ${${DEP}_INCLUDE_DIRS})
            endif()
            if(DEFINED ${DEP}_LIBRARIES})
                target_link_libraries(${TARGET} PUBLIC ${${DEP}_LIBRARIES})
            endif()
        endforeach()
    endif()
endfunction()

add_executable(fake_drone src/fake_drone.cc)
ament_target_dependencies(fake_drone rclcpp std_msgs nav_msgs geometry_msgs)

ament_export_dependencies(rclcpp std_msgs nav_msgs geometry_msgs)
ament_package()
