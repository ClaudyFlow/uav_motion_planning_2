#!/usr/bin/env python3
# ======= ROS1 VERSION (single_run_in_sim.launch) =======
# <launch>
#   <arg name="odom_topic" default="visual_slam/odom" />
#   <arg name="map_size_x" default="40.0" />
#   <arg name="map_size_y" default="20.0" />
#   <arg name="map_size_z" default="5.0" />
#   <arg name="init_x" value="-19.0" />
#   <arg name="init_y" value="0.0" />
#   <arg name="init_z" value="1.0" />
#   <node pkg="rviz" type="rviz" name="rviz" args="-d $(find plan_manage)/config/default.rviz" required="true" />
#   <include file="$(find plan_manage)/launch/simulator.xml">
#     <arg name="odom_topic" value="$(arg odom_topic)" />
#     <arg name="map_size_x_" value="$(arg map_size_x)" />
#     <arg name="map_size_y_" value="$(arg map_size_y)" />
#     <arg name="map_size_z_" value="$(arg map_size_z)" />
#     <arg name="init_x" value="$(arg init_x)" />
#     <arg name="init_y" value="$(arg init_y)" />
#     <arg name="init_z" value="$(arg init_z)" />
#   </include>
# </launch>

# ======= ROS2 VERSION =======
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    odom_topic = LaunchConfiguration('odom_topic', default='visual_slam/odom')
    map_size_x = LaunchConfiguration('map_size_x', default='40.0')
    map_size_y = LaunchConfiguration('map_size_y', default='20.0')
    map_size_z = LaunchConfiguration('map_size_z', default='5.0')
    init_x = LaunchConfiguration('init_x', default='-19.0')
    init_y = LaunchConfiguration('init_y', default='0.0')
    init_z = LaunchConfiguration('init_z', default='1.0')

    pkg_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    simulator_launch = os.path.join(pkg_dir, 'plan_manage', 'launch', 'simulator.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('odom_topic', default_value=odom_topic),
        DeclareLaunchArgument('map_size_x', default_value=map_size_x),
        DeclareLaunchArgument('map_size_y', default_value=map_size_y),
        DeclareLaunchArgument('map_size_z', default_value=map_size_z),
        DeclareLaunchArgument('init_x', default_value=init_x),
        DeclareLaunchArgument('init_y', default_value=init_y),
        DeclareLaunchArgument('init_z', default_value=init_z),

        # rviz
        Node(package='rviz2', executable='rviz2', name='rviz',
             arguments=['-d', os.path.join(pkg_dir, 'plan_manage', 'config', 'default.rviz')],
             condition=None),  # set condition if needed

        # simulator
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(simulator_launch),
            launch_arguments={
                'odom_topic': odom_topic,
                'map_size_x_': map_size_x,
                'map_size_y_': map_size_y,
                'map_size_z_': map_size_z,
                'init_x': init_x,
                'init_y': init_y,
                'init_z': init_z,
            }.items(),
        ),
    ])
