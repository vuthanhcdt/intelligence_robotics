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


def generate_launch_description():

    # Camera model (force value)
    left_node_name = 'left'
    left_camera_model = 'zedx'  # Camera model (force value)
    left_serial= '40726998'  
    left_zed_id = '0'
    left_publish_tf = 'true'
    left_publish_map_tf = 'true'
    left_cam_poses='[0.2,0.0,0.6,0.0,0.0,0.0]'


    # ZED Wrapper node
    zed_wrapper_launch = IncludeLaunchDescription(
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
            'publish_map_tf': left_publish_map_tf,
            'cam_pose': left_cam_poses 
        }.items()
    )
    
    # Define LaunchDescription variable
    ld = LaunchDescription()
    
    # Add nodes to LaunchDescription
    ld.add_action(zed_wrapper_launch)

    return ld
