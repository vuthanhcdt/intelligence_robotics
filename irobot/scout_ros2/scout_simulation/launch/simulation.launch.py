import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration
from launch_ros.actions import Node
import os
from pathlib import Path

MY_ROBOT = os.environ.get('ROBOT', "scout_mini")
MY_ENVIRONMENT = os.environ.get('ENV', "warehouse_env_walk_actor2")

def generate_launch_description():
    default_world_path = os.path.join(get_package_share_directory('scout_simulation'), 'worlds', MY_ENVIRONMENT + '.world')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    robot_dir = LaunchConfiguration(
        'robot_dir',
        default=os.path.join(get_package_share_directory('scout_simulation'),
            'robots/'+MY_ROBOT,
            MY_ROBOT+'.urdf'))

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')

    urdf = os.path.join(get_package_share_directory('scout_simulation'), 'robots/'+MY_ROBOT+'/', MY_ROBOT+'.urdf')

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        # arguments=['-entity', MY_ROBOT, '-file', urdf,"-x", "3.0", "-y", "0.0", "-z", "0.0","-R","0.0","-P","0.0","-Y","2.375"],
        arguments=['-entity', MY_ROBOT, '-file', urdf,"-x", "0.0", "-y", "0.0", "-z", "0.0","-R","0.0","-P","0.0","-Y","1.5732"],
        output='screen',
        )

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[urdf])

    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={
                'world': default_world_path
            }.items()
        )
    
    config_rviz = os.path.join(
        get_package_share_directory('scout_simulation'),
        'rviz',
        'scout_simulation.rviz'
        )
        
    # Rviz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='scout_simulation',
        output='screen',
        arguments=[["-d"], [config_rviz]],
    )

    return LaunchDescription([spawn_entity, start_robot_state_publisher_cmd, gazebo])
