#!/usr/bin/env python3

# Author: Anindya Mishra

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import ThisLaunchFileDir

def generate_launch_description():
    LDS_MODEL = os.environ.get('LDS_MODEL', 'LDS-02')  # fallback default if not set

    # Declare common launch arguments
    start_rviz = LaunchConfiguration('start_rviz')
    prefix = LaunchConfiguration('prefix')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    namespace = LaunchConfiguration('namespace')

    # Common launch description list
    launch_descriptions = [
        DeclareLaunchArgument(
            'start_rviz',
            default_value='false',
            description='Whether to execute rviz2'
        ),
        DeclareLaunchArgument(
            'prefix',
            default_value='""',
            description='Prefix of the joint and link names'
        ),
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='false',
            description='Start robot with fake hardware mirroring command to its states.'
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='m1',
            description='Namespace for the lidar node'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/m1_base.launch.py']),
            launch_arguments={
                'start_rviz': start_rviz,
                'prefix': prefix,
                'use_fake_hardware': use_fake_hardware,
            }.items(),
        )
    ]

    # Conditionally include the lidar launch based on LDS_MODEL
    if LDS_MODEL == 'LDS-01':
        launch_descriptions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('hls_lfcd_lds_driver'),
                        'launch',
                        'hlds_laser.launch.py'
                    ])
                ),
                launch_arguments={
                    'port': '/dev/ttyUSB0',
                    'frame_id': 'base_scan',
                }.items(),
            )
        )
    elif LDS_MODEL == 'LDS-02':
        launch_descriptions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('ld08_driver'),
                        'launch',
                        'ld08.launch.py'
                    ])
                ),
                launch_arguments={
                    'port': '/dev/ttyUSB0',
                    'frame_id': 'base_scan',
                    'namespace': namespace
                }.items(),
            )
        )
    else:
        raise RuntimeError(f"Unsupported LDS_MODEL value: {LDS_MODEL}")

    return LaunchDescription(launch_descriptions)
