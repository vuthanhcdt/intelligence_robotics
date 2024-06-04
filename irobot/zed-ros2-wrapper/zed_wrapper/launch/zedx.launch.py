import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Camera model (force value)
    left_node_name = 'left'
    left_camera_model = 'zedx'  # Camera model (force value)
    left_serial= '40885357'  
    left_zed_id = '0'
    left_publish_tf = 'false'
    left_publish_map_tf = 'true'


    # ZED Wrapper node
    zed_wrapper_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('zed_wrapper'),
            '/launch/zed_camera.launch.py'
        ]),
        launch_arguments={
            'node_name': left_node_name,
            'camera_name': left_camera_model,
            'camera_model': left_camera_model,
            'publish_tf': left_publish_tf,
            'publish_map_tf': left_publish_map_tf,
        }.items()
    )
    
    # Define LaunchDescription variable
    ld = LaunchDescription()
    
    # Add nodes to LaunchDescription
    ld.add_action(zed_wrapper_launch)

    return ld
