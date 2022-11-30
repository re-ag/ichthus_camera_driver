# Copyright 2022. Jaeun Kim (763k357@gmail.com) and Kanghee Kim(kim.kanghee@gmail.com) all rights reserved.
# added by ICHTHUS, Jaeeun Kim on 20221130

import os
from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
import yaml
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    
    ichthus_camera_driver = Node(
            package='ichthus_camera_driver',
            executable='ichthus_camera_driver_node',
            name='ichthus_camera_driver',
            parameters=[
                {
                    "width": 960,
                    "height": 640,
                    "rate" : 13,
                }
            ],
            output="screen"
        )

    return [ichthus_camera_driver]


def generate_launch_description():

    launch_arguments = []

    def add_launch_arg(name: str, default_value=None):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value))

    return launch.LaunchDescription(
        launch_arguments 
        + [OpaqueFunction(function=launch_setup)]
    )