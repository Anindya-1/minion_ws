#!/usr/bin/env python3

# Authors: Will Son


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Declare a launch argument for the namespace
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='m1',
        description='Namespace for the robot'
    )

    return LaunchDescription([
        Node(
            package='ld08_driver',
            executable='ld08_driver',
            name='ld08_driver',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            remappings=[
                ('/scan', 'scan')  # Remap /scan to <namespace>/scan
            ]
        ),
    ])
