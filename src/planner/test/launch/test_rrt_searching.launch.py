#!/usr/bin/env python3
# ======= ROS1 VERSION =======
# <node name="test_rrt_searching" pkg="test" type="test_rrt_searching" output="screen">
#   <param name="grid_map/resolution" value="0.1" />
#   ...
# </node>

# ======= ROS2 VERSION =======
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='test',
            executable='test_rrt_searching',
            name='test_rrt_searching',
            output='screen',
            parameters=[{
                'grid_map/resolution': 0.1,
                'grid_map/map_size_x': 40.0,
                'grid_map/map_size_y': 20.0,
                'grid_map/map_size_z': 3.0,
                'grid_map/local_update_range_x': 40.0,
                'grid_map/local_update_range_y': 20.0,
                'grid_map/local_update_range_z': 3.0,
                'grid_map/obstacles_inflation': 0.099,
                'grid_map/local_map_margin': 30,
                'grid_map/ground_height': -0.01,
                'grid_map/cx': 321.04638671875,
                'grid_map/cy': 243.44969177246094,
                'grid_map/fx': 387.229248046875,
                'grid_map/fy': 387.229248046875,
            }],
        ),
    ])
