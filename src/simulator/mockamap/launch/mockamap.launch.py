#!/usr/bin/env python3
# ======= ROS1 VERSION =======
# <node pkg="mockamap" type="mockamap_node" name="mockamap_node" output="screen">
#   <param name="seed" type="int" value="511" />
#   ...
# </node>

# ======= ROS2 VERSION =======
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mockamap',
            executable='mockamap_node',
            name='mockamap_node',
            output='screen',
            parameters=[{
                'seed': 511,
                'update_freq': 1.0,
                'resolution': 0.1,
                'x_length': 20,
                'y_length': 20,
                'z_length': 20,
                'type': 4,
            }],
        ),
    ])
