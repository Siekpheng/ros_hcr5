import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, MotionPlanRequest
from shape_msgs.msg import SolidPrimitive
from cv_bridge import CvBridge
import cv2
import numpy as np
import os

class HCR5UnifiedPicker(Node):
    def __init__(self):
        super().__init__('hcr5_unified_picker')
        
        # --- 1. Vision Setup ---
        self.bridge = CvBridge()
        self.latest_coords = None
        self.offset_x = -0.005  # Adjust for accuracy tuning
        self.offset_y = +0.015
        
        # Load Calibration Matrix
        script_dir = os.path.dirname(os.path.realpath(__file__))
        file_path = os.path.join(script_dir, "table_calibration.npy")
        try:
            self.M = np.load(file_path)
            self.get_logger().info(f"Calibration loaded from: {file_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load calibration: {e}")
            self.M = None

        # --- 2. Motion Setup ---
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        
        # --- 3. ROS Subscriptions ---
        self.sub = self.create_subscription(Image, 'camera1/image_raw', self.image_callback, 10)
        
        self.get_logger().info("Unified Picker Online. Press SPACE in the camera window to move.")

    def image_callback(self, msg):
        if self.M is None: return
        
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Detect Purple Object
        mask = cv2.inRange(hsv, np.array([130, 50, 50]), np.array([160, 255, 255]))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        self.latest_coords = None # Reset every frame
        for cnt in contours:
            if cv2.contourArea(cnt) > 500:
                Moments = cv2.moments(cnt)
                if Moments["m00"] != 0:
                    u = int(Moments["m10"] / Moments["m00"])
                    v = int(Moments["m01"] / Moments["m00"])

                    # Homography Transform
                    pixel_pt = np.array([u, v, 1], dtype="float32")
                    world_pt = np.dot(self.M, pixel_pt)
                    raw_x = world_pt[0] / world_pt[2]
                    raw_y = world_pt[1] / world_pt[2]
                    
                    self.latest_coords = (raw_x + self.offset_x, raw_y + self.offset_y)

                    # Visual Overlays
                    cv2.circle(frame, (u, v), 10, (255, 0, 255), -1)
                    cv2.putText(frame, f"X:{self.latest_coords[0]:.2f} Y:{self.latest_coords[1]:.2f}", 
                                (u+15, v), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        cv2.imshow("Unified Vision Feed", frame)
        
        # Check for SPACEBAR in the OpenCV window
        key = cv2.waitKey(1) & 0xFF
        if key == ord(' '):
            if self.latest_coords:
                self.send_move_command(self.latest_coords[0], self.latest_coords[1])
            else:
                self.get_logger().warn("No object detected!")

    def send_move_command(self, x, y):
        self.get_logger().info(f"Sending Goal: X={x:.3f}, Y={y:.3f}")
        
        goal_msg = MoveGroup.Goal()
        request = MotionPlanRequest()
        request.group_name = "arm" 
        request.num_planning_attempts = 10
        request.allowed_planning_time = 5.0
        request.pipeline_id = "ompl"

        constraints = Constraints()
        
        # Position Constraint
        pos_con = PositionConstraint()
        pos_con.header.frame_id = "base_link"
        pos_con.link_name = "link6_1"  # Verify this in RViz/URDF
        
        bounding_box = SolidPrimitive()
        bounding_box.type = SolidPrimitive.BOX
        bounding_box.dimensions = [0.02, 0.02, 0.02] # 2cm tolerance zone
        
        target_pose = PoseStamped()
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = 0.15 # Approach height
        
        pos_con.constraint_region.primitives.append(bounding_box)
        pos_con.constraint_region.primitive_poses.append(target_pose.pose)
        pos_con.weight = 1.0

        # Orientation Constraint (Pointing Down)
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
        self._action_client.send_goal_async(goal_msg)

def main():
    rclpy.init()
    node = HCR5UnifiedPicker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()