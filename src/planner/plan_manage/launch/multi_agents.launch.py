#!/usr/bin/env python3
# ======= ROS1 VERSION (multi_agents.launch) =======
# <launch>
#   <include file="$(find plan_manage)/launch/single_run_in_sim.launch" ns="cf0">
#     <arg name="namespace" value="cf0" />
#   </include>
#   <include file="$(find plan_manage)/launch/single_run_in_sim.launch" ns="cf1">
#     <arg name="namespace" value="cf1" />
#   </include>
#   ...
# </launch>

# ======= ROS2 VERSION =======
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    pkg_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    single_launch = os.path.join(pkg_dir, 'plan_manage', 'launch', 'single_run_in_sim.launch.py')

    agents = ['cf0', 'cf1', 'cf2', 'cf3', 'cf4', 'cf5']

    actions = []
    for agent in agents:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(single_launch),
                launch_arguments={
                    'namespace': agent,
                }.items(),
            )
        )

    return LaunchDescription(actions)
