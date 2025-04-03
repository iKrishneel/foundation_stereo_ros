#!/usr/bin/env python

import os.path as osp

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_package = 'foundation_stereo_ros'

def launch_setup(context, *args, **kwargs):
    param_file = LaunchConfiguration('param_file')
    assert osp.isfile(param_file.perform(context)), f'{param_file.perform(context)} not found!'
    
    ns, left_topic, right_topic, info_topic, baseline, config, pub_pcd, weights = [
        LaunchConfiguration(name) for name in ['ns', 'left', 'right', 'info', 'baseline', 'config', 'pub_pcd', 'weights']
    ]

    stereo_node = Node(
        package=_package,
        executable='foundation_stereo_node.py',
        name='foundation_stereo',
        remappings=[('left', left_topic), ('right', right_topic), ('info', info_topic)],
        parameters=[{'config': config, 'pub_pub': pub_pcd, 'weights': weights}, param_file],
        respawn=False,
        output='both',
        namespace=ns,
    )

    return [stereo_node]


def generate_launch_description():
    param_file = osp.join(get_package_share_directory(_package), 'launch/', 'default_params.yaml')
    config = osp.join(get_package_share_directory(_package), 'launch/configs', 'foundation_stereo.yaml ')
    declared_args = [
        DeclareLaunchArgument('param_file', default_value=param_file, description='Parameter file'),
        DeclareLaunchArgument('ns', default_value='/', description='Namespace'),
        DeclareLaunchArgument('left', default_value='/camera/left/image_raw', description='Left camera image topic'),
        DeclareLaunchArgument('right', default_value='/camera/right/image_raw', description='Right camera image topic'),
        DeclareLaunchArgument('info', default_value='/camera/right/camera_info', description='Left camera info topic'),
        DeclareLaunchArgument('baseline', default_value='0.12', description='Baseline between two camera in meters'),
        DeclareLaunchArgument('config', default_value=config, description='Igniter config file'),
        DeclareLaunchArgument('pub_pcd', default_value=config, description='Publish point cloud'),
        DeclareLaunchArgument('weights', default_value='', description='Foundation stereo model weights'),
    ]

    return LaunchDescription(declared_args + [OpaqueFunction(function=launch_setup)])
