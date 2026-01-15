import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Point, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, MotionPlanRequest
from shape_msgs.msg import SolidPrimitive

class HCR5MotionController(Node):
    def __init__(self):
        super().__init__('hcr5_motion_controller')
        
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        
        self.subscription = self.create_subscription(
            Point,
            '/target_coordinates',
            self.move_to_callback,
            10)
        
        self.get_logger().info("Motion Controller Online. Waiting for SPACEBAR trigger...")

    def move_to_callback(self, msg):
        self.get_logger().info(f"Preparing to move to: X={msg.x:.2f}, Y={msg.y:.2f}")
        
        goal_msg = MoveGroup.Goal()
        request = MotionPlanRequest()
        request.group_name = "arm" 
        request.num_planning_attempts = 10
        request.allowed_planning_time = 5.0
        request.pipeline_id = "ompl"
        
        # 1. Setup Constraints
        constraints = Constraints()
        
        # Position Constraint
        pos_con = PositionConstraint()
        pos_con.header.frame_id = "base_link"
        pos_con.link_name = "link6_1" # DOUBLE CHECK THIS LINK NAME
        
        # Define a small bounding box around the target point as the goal
        bounding_box = SolidPrimitive()
        bounding_box.type = SolidPrimitive.BOX
        bounding_box.dimensions = [0.01, 0.01, 0.01] # 1cm tolerance
        
        pos_con.constraint_region.primitives.append(bounding_box)
        
        # Set the target position
        target_pose = PoseStamped()
        target_pose.pose.position.x = msg.x
        target_pose.pose.position.y = msg.y
        target_pose.pose.position.z = 0.15
        pos_con.constraint_region.primitive_poses.append(target_pose.pose)
        pos_con.weight = 1.0
        
        # Orientation Constraint (Facing Down)
        ori_con = OrientationConstraint()
        ori_con.header.frame_id = "base_link"
        ori_con.link_name = "link6_1"
        ori_con.orientation.x = 0.0
        ori_con.orientation.y = 1.0 
        ori_con.orientation.z = 0.0
        ori_con.orientation.w = 0.0
        ori_con.absolute_x_axis_tolerance = 0.1
        ori_con.absolute_y_axis_tolerance = 0.1
        ori_con.absolute_z_axis_tolerance = 0.1
        ori_con.weight = 1.0

        constraints.position_constraints.append(pos_con)
        constraints.orientation_constraints.append(ori_con)
        
        request.goal_constraints.append(constraints)
        goal_msg.request = request
        goal_msg.planning_options.plan_only = False 
        
        self._action_client.wait_for_server()
        self.get_logger().info("Sending goal...")
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

def main():
    rclpy.init()
    node = HCR5MotionController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()