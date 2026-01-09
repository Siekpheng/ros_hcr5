from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("hcr5", package_name="hcr5_moveit_config")
        .to_moveit_configs()
    )

    # Manually define the node but add the publishing flags
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": True,
                "publish_robot_description": True,
                "publish_robot_description_semantic": True,
            },
        ],
    )

    return LaunchDescription([move_group_node])