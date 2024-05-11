import os
import launch
import launch_ros

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to the parameters YAML file
    config = os.path.join(
        get_package_share_directory('visual_control_v1'),
        'config',
        'params.yaml'
    )

    # Local planning node configuration
    node = launch_ros.actions.Node(
        name='pid_control',
        package='visual_control_v1',
        executable='pid_control.py',
        output='screen',
        emulate_tty=True,
        parameters=[config]
    )
    
    # Return the LaunchDescription with both nodes
    return LaunchDescription([
        node
    ])
