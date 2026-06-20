import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[{
                'use_sim_time': True,
                'resolution': 0.05,
                'max_laser_range': 3.5,
                'minimum_travel_distance': 0.1,
                'minimum_travel_heading': 0.1,
                'scan_buffer_size': 10,
                'scan_topic': '/scan',
                'base_frame': 'base_footprint',
                'odom_frame': 'odom',
                'map_frame': 'map',
            }],
            output='screen'
        ),
    ])