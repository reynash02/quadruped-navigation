#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # Include the RTAB-Map launch file with remappings
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('rtabmap_launch'),
                'launch',
                'rtabmap.launch.py'
            ])
        ]),
        launch_arguments={
            'rtabmap_args': '--delete_db_on_start',
            'rgb_topic': '/camera/color/image_raw',
            'depth_topic': '/camera/aligned_depth_to_color/image_raw',
            'camera_info_topic': '/camera/color/camera_info',
            'depth_camera_info_topic': '/camera/aligned_depth_to_color/camera_info',
            'frame_id': 'base_link',
            'approx_sync': 'true',
            'use_sim_time': 'true',
            'odom_topic': '/odom',
            'visual_odometry': 'false',  # Use /odom from your robot
            'rviz': 'false',
        }.items()
    )
    
    return LaunchDescription([
        rtabmap_launch,
    ])