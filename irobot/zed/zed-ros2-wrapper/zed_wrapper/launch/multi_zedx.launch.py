# Copyright 2022 Stereolabs
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnExecutionComplete
from launch.actions import RegisterEventHandler
from launch_ros.actions import Node
import launch_ros


def generate_launch_description():

    # left camera parameters
    left_node_name = 'left'
    left_camera_model = 'zedx'  # Camera model (force value)
    left_serial= '40885357'  
    left_zed_id = '0'
    left_publish_tf = 'false'
    left_publish_map_tf = 'false'
    # left_cam_poses='[0.265,0.165,0.6,0.0,0.0,1.0995574288]'# 63
    left_cam_poses='[0.265,0.165,0.6,0.0,0.0,0.698132]'# 40

     # right camera parameters
    right_node_name = 'right'
    right_camera_model = 'zedx'  # Camera model (force value)
    right_serial= '43870948'  
    right_zed_id = '1'
    right_publish_tf = 'false'
    right_publish_map_tf = 'false'
    # right_cam_poses='[0.265,0.165,0.6,0.0,0.0,1.0995574288]'# 63
    right_cam_poses='[0.265,0.165,0.6,0.0,0.0,0.698132]'# 40

    # ZED Wrapper node for left camera
    left_zed_wrapper_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('zed_wrapper'),
            '/launch/include/zed_camera.launch.py'
        ]),
        launch_arguments={
            'node_name': left_node_name,
            'camera_model': left_camera_model,
            'serial_number': left_serial,
            'zed_id': left_zed_id,
            'publish_tf': left_publish_tf,
            'publish_map_tf': left_publish_map_tf
        }.items()
    )

   # ZED Wrapper node for right camera
    right_zed_wrapper_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('zed_wrapper'),
            '/launch/include/zed_camera.launch.py'
        ]),
        launch_arguments={
            'node_name': right_node_name,
            'camera_model': right_camera_model,
            'serial_number': right_serial,
            'zed_id': right_zed_id,
            'publish_tf': right_publish_tf,
            'publish_map_tf': right_publish_map_tf          
        }.items()
    )

    static_tf2_right= Node(package="tf2_ros",
            executable="static_transform_publisher",
            output="screen" ,
            arguments=["0.265", "-0.18", "0.6", "-0.5236", "0.0", "0", "base_link", "zedx_right_base_link"]
        )
    static_tf2_left= Node(package="tf2_ros",
            executable="static_transform_publisher",
            output="screen" ,
            arguments=["0.265", " 0.18", "0.6", " 0.5236", "0.0", "0", "base_link", "zedx_left_base_link"]
        )

    return LaunchDescription([
        right_zed_wrapper_launch,
        left_zed_wrapper_launch,
        static_tf2_right,
        static_tf2_left
    ])
