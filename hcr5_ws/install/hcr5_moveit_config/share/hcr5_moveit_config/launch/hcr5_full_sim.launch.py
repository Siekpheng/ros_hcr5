import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. Setup paths
    moveit_config_path = FindPackageShare('hcr5_moveit_config')
    description_path = FindPackageShare('hcr5_description')

    # 2. Launch Gazebo (The Body)
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([description_path, 'launch', 'gazebo.launch.py'])
        ]),
        launch_arguments={'use_sim_time': 'true'}.items() # Add this!
    )

    # 3. The Clock Bridge (The Heartbeat)
    # This specifically fixes your "time diff" errors
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # 4. MoveGroup (The Brain)
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([moveit_config_path, 'launch', 'move_group.launch.py'])
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # 5. RViz (The Eyes)
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([moveit_config_path, 'launch', 'moveit_rviz.launch.py'])
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    return LaunchDescription([
        gazebo_sim,
        clock_bridge,
        move_group,
        rviz
    ])

