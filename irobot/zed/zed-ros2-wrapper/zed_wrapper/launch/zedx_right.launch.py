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
    right_node_name = 'right'
    right_camera_model = 'zedx'  # Camera model (force value)
    right_serial= '43870948'  
    right_zed_id = '1'
    right_publish_tf = 'false'
    right_publish_map_tf = 'false'

    # ZED Wrapper node
    zed_wrapper_launch = IncludeLaunchDescription(
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
    
    # Define LaunchDescription variable
    ld = LaunchDescription()
    
    # Add nodes to LaunchDescription
    ld.add_action(zed_wrapper_launch)

    return ld
