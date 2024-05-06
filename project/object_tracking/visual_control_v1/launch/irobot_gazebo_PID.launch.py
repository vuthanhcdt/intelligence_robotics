import os
import launch
import launch_ros

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():

    node_name='irobot_gazebo_PID'
    
    # Get the path to the parameters YAML file
    config = os.path.join(
        get_package_share_directory('visual_control_v1'),
        'config',
        'params.yaml'
    )
    
    # Get the path to the RViz configuration file
    config_rviz2 = os.path.join(
        get_package_share_directory('visual_control_v1'),
        'rviz2',
        'visual_control_v1.rviz'
    )

    # RViz2 node configuration
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='visual_control_v1',
        output='screen',
        arguments=[["-d"], [config_rviz2]],
    )

    # Local planning node configuration
    node = launch_ros.actions.Node(
        name=node_name,
        package='visual_control_v1',
        executable='irobot_gazebo_PID.py',
        output='screen',
        emulate_tty=True,
        parameters=[config]
    )
    
    # Return the LaunchDescription with both nodes
    return LaunchDescription([
        node
    ])
