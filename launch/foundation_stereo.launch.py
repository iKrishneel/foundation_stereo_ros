#!/usr/bin/env python

import os
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
    
    ns, left_topic, right_topic, info_topic, baseline, config, weights, depth_scale, depth, points = [
        LaunchConfiguration(name) for name in [
            'ns', 'left', 'right', 'info', 'baseline', 'config', 'weights', 'depth_scale', 'depth', 'points'
        ]
    ]

    pub_pcd = False
    remappings = [('left', left_topic), ('right', right_topic), ('info', info_topic), ('depth', depth)]
    if len(points.perform(context)) > 0:
        remappings.append(('points', points))
        pub_pcd = True

    stereo_node = Node(
        package=_package,
        executable='foundation_stereo_node.py',
        name='foundation_stereo',
        remappings=remappings,
        parameters=[{'config': config, 'pub_pcd': pub_pcd, 'weights': weights, 'depth_scale': depth_scale}, param_file],
        respawn=False,
        output='both',
        namespace=ns,
    )

    return [stereo_node]


def generate_launch_description():
    param_file = osp.join(get_package_share_directory(_package), 'launch/', 'default_params.yaml')
    config = osp.join(get_package_share_directory(_package), 'launch/configs', 'foundation_stereo.yaml ')
    weights = osp.join(os.environ['HOME'], '.cache/torch/model_best_bp2.pth')
    weights = weights if osp.isfile(weights) else ''

    declared_args = [
        DeclareLaunchArgument('param_file', default_value=param_file, description='Parameter file'),
        DeclareLaunchArgument('ns', default_value='/', description='Namespace'),
        DeclareLaunchArgument('left', default_value='/camera/left/image_raw', description='Left camera image topic'),
        DeclareLaunchArgument('right', default_value='/camera/right/image_raw', description='Right camera image topic'),
        DeclareLaunchArgument('info', default_value='/camera/right/camera_info', description='Left camera info topic'),
        DeclareLaunchArgument('depth', default_value='/depth', description='Output depth topic'),
        DeclareLaunchArgument('points', default_value='', description='Output points topic'),
        DeclareLaunchArgument('baseline', default_value='0.12', description='Baseline between two camera in meters'),
        DeclareLaunchArgument('config', default_value=config, description='Igniter config file'),
        # DeclareLaunchArgument('pub_pcd', default_value='False', description='Publish point cloud'),
        DeclareLaunchArgument('weights', default_value=weights, description='Foundation stereo model weights'),
        DeclareLaunchArgument('depth_scale', default_value='1.0', description='Depth scaling factor'),
    ]

    return LaunchDescription(declared_args + [OpaqueFunction(function=launch_setup)])
