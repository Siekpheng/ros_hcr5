#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

# Standard ROS 2 MoveIt Messages
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume
from geometry_msgs.msg import Pose, Point, Quaternion
from shape_msgs.msg import SolidPrimitive

class MoveRobotClient(Node):
    def __init__(self):
        super().__init__('move_robot_client')
        self._action_client = ActionClient(self, MoveGroup, 'move_action')

    def send_goal(self, x, y, z):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        # 1. Create the Goal Message
        goal_msg = MoveGroup.Goal()
        goal_msg.request.workspace_parameters.header.frame_id = 'world'
        goal_msg.request.workspace_parameters.min_corner.x = -1.0
        goal_msg.request.workspace_parameters.min_corner.y = -1.0
        goal_msg.request.workspace_parameters.min_corner.z = -1.0
        goal_msg.request.workspace_parameters.max_corner.x = 1.0
        goal_msg.request.workspace_parameters.max_corner.y = 1.0
        goal_msg.request.workspace_parameters.max_corner.z = 1.0
        
        goal_msg.request.start_state.is_diff = True
        goal_msg.request.group_name = 'arm' # CHECK YOUR SRDF GROUP NAME
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 1.0
        goal_msg.request.max_acceleration_scaling_factor = 1.0
        
        # 2. Create Constraints (Target Pose)
        # Position Constraint
        pcm = PositionConstraint()
        pcm.header.frame_id = 'base_link'
        pcm.link_name = 'link6_1' # End effector link name
        pcm.constraint_region.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.01, 0.01, 0.01]))
        
        # Target Position
        target_point = Point(x=x, y=y, z=z)
        pcm.constraint_region.primitive_poses.append(Pose(position=target_point))
        pcm.weight = 1.0

        # Orientation Constraint (Optional but recommended to keep hand level)
        ocm = OrientationConstraint()
        ocm.header.frame_id = 'base_link'
        ocm.link_name = 'link6_1'
        ocm.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0) # Flat/Forward
        ocm.absolute_x_axis_tolerance = 0.1
        ocm.absolute_y_axis_tolerance = 0.1
        ocm.absolute_z_axis_tolerance = 0.1
        ocm.weight = 1.0

        # Add constraints to goal
        goal_msg.request.goal_constraints.append(Constraints(name='', position_constraints=[pcm], orientation_constraints=[ocm]))
        goal_msg.planning_options.plan_only = False # Set to True to only visualize
        goal_msg.planning_options.look_around = False
        goal_msg.planning_options.replan = True

        self.get_logger().info(f'Sending goal to x={x} y={y} z={z}...')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted! Moving...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.error_code.val == 1: # 1 = SUCCESS
            self.get_logger().info('Success! Robot Moved.')
        else:
            self.get_logger().info(f'Movement failed with error code: {result.error_code.val}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    client = MoveRobotClient()
    
    # Target Coordinates (Adjust as safe)
    client.send_goal(0.1, -0.6, 0.5) 
    
    rclpy.spin(client)

if __name__ == '__main__':
    main()