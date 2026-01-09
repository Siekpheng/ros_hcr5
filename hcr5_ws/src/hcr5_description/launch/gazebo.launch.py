import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Paths
    pkg_description = get_package_share_directory("hcr5_description")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    
    xacro_file = os.path.join(pkg_description, "urdf", "hcr5.xacro")

    # 2. Process Xacro
    robot_description_content = xacro.process_file(xacro_file).toxml()

    # 3. Environment Variable for Meshes
    set_gz_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[os.path.join(pkg_description, '..')]
    )

    # 4. Gazebo Sim Launch
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={'gz_args': '-r sensors.sdf'}.items(),
    )

    # 5. The Clock Bridge (CRITICAL: Added back into the logic)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
        ],
        output='screen'
    )

    # 6. Nodes
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "use_sim_time": True, 
            "robot_description": robot_description_content
        }],
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description", "-name", "hcr5"],
        output="screen",
        parameters=[{"use_sim_time": True}] # Added for sync
    )

    # 7. Controller Spawners (Ensure they use sim time too)
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        parameters=[{"use_sim_time": True}]
    )

    arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
        parameters=[{"use_sim_time": True}]
    )

    return LaunchDescription([
        set_gz_resource_path,
        gz_sim,
        bridge,  # <--- BRIDGE ADDED HERE
        robot_state_publisher,
        spawn_entity,
        joint_state_broadcaster,
        arm_controller
    ])