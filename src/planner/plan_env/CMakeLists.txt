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





cmake_minimum_required(VERSION 3.10)





project(plan_env)





if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")


  add_compile_options(-Wall -Wextra -Wpedantic)


endif()





find_package(ament_cmake REQUIRED)


find_package(rclcpp REQUIRED)


find_package(std_msgs REQUIRED)


find_package(service_msgs REQUIRED)


find_package(Eigen3 REQUIRED)


find_package(OpenCV REQUIRED)


find_package(geometry_msgs REQUIRED)





include(CMakeParseArguments)


function(ament_target_dependencies TARGET)


    cmake_parse_arguments(ATD "" "" "DEPENDENCIES" ${ARGN})


    if(ATD_DEPENDENCIES)


        foreach(DEP ${ATD_DEPENDENCIES})


            find_package(${DEP} REQUIRED)


            if(DEFINED ${DEP}_INCLUDE_DIRS)


                target_include_directories(${TARGET} PUBLIC ${${DEP}_INCLUDE_DIRS})


            endif()


            if(DEFINED ${DEP}_LIBRARIES)


                target_link_libraries(${TARGET} PUBLIC ${${DEP}_LIBRARIES})


            endif()


        endforeach()


    endif()


endfunction()





file(GLOB_RECURSE PLAN_ENV_SOURCES "src/plan_env/*.cc")





add_library(plan_env ${PLAN_ENV_SOURCES})





ament_target_dependencies(plan_env rclcpp std_msgs service_msgs Eigen3 cv_bridge geometry_msgs visualization_msgs sensor_msgs)





# 确保 rclcpp 和 std_msgs 的 include 目录被加入


target_include_directories(plan_env PUBLIC


  ${rclcpp_INCLUDE_DIRS}


  ${std_msgs_INCLUDE_DIRS}


  ${service_msgs_INCLUDE_DIRS}


  ${EIGEN3_INCLUDE_DIRS}


  /opt/ros/lyrical/include/service_msgs


  /opt/ros/lyrical/include/type_description_interfaces


  /opt/ros/lyrical/include/cv_bridge


  /opt/ros/lyrical/include/sensor_msgs
  /opt/ros/lyrical/include/pcl_conversions
  /opt/ros/lyrical/include/pcl_msgs
  /usr/include/pcl-1.15


  /usr/include/opencv4


  /opt/ros/lyrical/include
  /opt/ros/lyrical/include/geometry_msgs
  /opt/ros/lyrical/include/nav_msgs
  /opt/ros/lyrical/include/visualization_msgs
  /opt/ros/lyrical/include/sensor_msgs
  /opt/ros/lyrical/include/message_filters
  /opt/ros/lyrical/include/message_filters/message_filters


  include


  src


)





ament_export_dependencies(rclcpp std_msgs service_msgs Eigen3)
ament_package()


