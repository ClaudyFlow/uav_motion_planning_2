#!/usr/bin/env python3
# ======= ROS1 VERSION (simulator_example.launch) =======
# <launch>
#   <arg name="init_x" value="-4" />
#   <arg name="init_y" value="0" />
#   <arg name="init_z" value="4" />
#   <node pkg="so3_quadrotor_simulator" type="quadrotor_simulator_so3" name="quadrotor_simulator_so3" output="screen">
#     <param name="rate/odom" value="100.0" />
#     <param name="simulator/init_state_x" value="$(arg init_x)" />
#     ...
#     <remap from="~odom" to="/sim/odom" />
#     <remap from="~cmd" to="/so3_cmd" />
#     <remap from="~imu" to="/sim/imu" />
#   </node>
#   <!-- nodelet standalone so3_control/SO3ControlNodelet -->
# </launch>

# ======= ROS2 VERSION =======
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    init_x = LaunchConfiguration('init_x', default='-4')
    init_y = LaunchConfiguration('init_y', default='0')
    init_z = LaunchConfiguration('init_z', default='4')

    return LaunchDescription([
        DeclareLaunchArgument('init_x', default_value='-4'),
        DeclareLaunchArgument('init_y', default_value='0'),
        DeclareLaunchArgument('init_z', default_value='4'),

        # Simulator
        Node(package='so3_quadrotor_simulator', executable='quadrotor_simulator_so3',
             name='quadrotor_simulator_so3', output='screen',
             parameters=[{
                 'rate/odom': 100.0,
                 'simulator/init_state_x': init_x,
                 'simulator/init_state_y': init_y,
                 'simulator/init_state_z': init_z,
             }],
             remappings=[
                 ('odom', '/sim/odom'),
                 ('cmd', '/so3_cmd'),
                 ('imu', '/sim/imu'),
             ]),

        # Controller (NOTE: so3_control nodelet has no ROS2 equivalent - convert to component)
        # Node(package='so3_control', executable='so3_control_node', ...),
    ])
