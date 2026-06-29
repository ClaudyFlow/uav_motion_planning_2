#!/usr/bin/env python3
# ======= ROS1 VERSION (simulator.xml) =======
# <launch>
#   <arg name="odom_topic" />
#   <arg name="map_size_x_" />
#   <arg name="map_size_y_" />
#   <arg name="map_size_z_" />
#   <arg name="init_x" />
#   <arg name="init_y" />
#   <arg name="init_z" />
#   <node name="random_forest" pkg="map_generator" type="random_forest" output="screen">
#     <remap from="~odom" to="$(arg odom_topic)" />
#     <param name="init_state_x" value="$(arg init_x)" />
#     ...
#   </node>
#   ...
# </launch>

# ======= ROS2 VERSION =======
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetParameter
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    odom_topic = LaunchConfiguration('odom_topic')
    map_size_x_ = LaunchConfiguration('map_size_x_', default='40.0')
    map_size_y_ = LaunchConfiguration('map_size_y_', default='20.0')
    map_size_z_ = LaunchConfiguration('map_size_z_', default='5.0')
    init_x = LaunchConfiguration('init_x', default='-19.0')
    init_y = LaunchConfiguration('init_y', default='0.0')
    init_z = LaunchConfiguration('init_z', default='1.0')

    pkg_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

    return LaunchDescription([
        DeclareLaunchArgument('odom_topic'),
        DeclareLaunchArgument('map_size_x_', default_value='40.0'),
        DeclareLaunchArgument('map_size_y_', default_value='20.0'),
        DeclareLaunchArgument('map_size_z_', default_value='5.0'),
        DeclareLaunchArgument('init_x', default_value='-19.0'),
        DeclareLaunchArgument('init_y', default_value='0.0'),
        DeclareLaunchArgument('init_z', default_value='1.0'),

        # map_generator
        Node(package='map_generator', executable='random_forest', name='random_forest',
             output='screen',
             remappings=[('odom', odom_topic)],
             parameters=[{
                 'init_state_x': init_x,
                 'init_state_y': init_y,
                 'init_state_z': init_z,
                 'map/x_size': map_size_x_,
                 'map/y_size': map_size_y_,
                 'map/z_size': map_size_z_,
                 'map/resolution': 0.1,
                 'map/fix_map_type': 0,
                 'ObstacleShape/seed': 1,
                 'map/obs_num': 70,
                 'ObstacleShape/lower_rad': 0.5,
                 'ObstacleShape/upper_rad': 0.8,
             }]),

        # fake_drone
        Node(package='fake_drone', executable='fake_drone', name='fake_drone',
             output='screen',
             parameters=[{
                 'init_x': init_x,
                 'init_y': init_y,
                 'init_z': init_z,
             }]),
    ])
