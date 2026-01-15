import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from geometry_msgs.msg import Point  # Sending coordinates to the motion server

class HCR5Picker(Node):
    def __init__(self):
        super().__init__('hcr5_picker')
        self.bridge = CvBridge()
        
        # --- ACCURACY TUNING (In Meters) ---
        # If your accuracy is off, adjust these values!
        self.offset_x = -0.005 # e.g., -0.02 if the robot reaches 2cm too far
        self.offset_y = +0.015  # e.g., 0.01 if the robot reaches 1cm too far right
        
        # Load Calibration
        script_dir = os.path.dirname(os.path.realpath(__file__))
        file_path = os.path.join(script_dir, "table_calibration.npy")
        
        try:
            self.M = np.load(file_path)
            self.get_logger().info(f"Calibration loaded from: {file_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load calibration: {e}")
            self.M = None 

        # Publisher for the target location
        self.coord_pub = self.create_publisher(Point, '/target_coordinates', 10)
        self.sub = self.create_subscription(Image, 'camera1/image_raw', self.image_callback, 10)
        
        self.latest_world_coords = None

    def image_callback(self, msg):
        if self.M is None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Detect Purple Object (Target)
        lower_purple = np.array([130, 50, 50])
        upper_purple = np.array([160, 255, 255])
        mask = cv2.inRange(hsv, lower_purple, upper_purple)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for cnt in contours:
            if cv2.contourArea(cnt) > 500:
                Moments = cv2.moments(cnt)
                if Moments["m00"] != 0:
                    u = int(Moments["m10"] / Moments["m00"])
                    v = int(Moments["m01"] / Moments["m00"])

                    # 1. Homography Transform
                    pixel_pt = np.array([u, v, 1], dtype="float32")
                    world_pt = np.dot(self.M, pixel_pt)
                    
                    # 2. Normalize and Apply Offsets
                    raw_x = world_pt[0] / world_pt[2]
                    raw_y = world_pt[1] / world_pt[2]
                    
                    self.latest_world_coords = (raw_x + self.offset_x, raw_y + self.offset_y)

                    # 3. Visual Overlays
                    cv2.circle(frame, (u, v), 10, (255, 0, 255), -1)
                    text = f"X:{self.latest_world_coords[0]:.2f} Y:{self.latest_world_coords[1]:.2f}"
                    cv2.putText(frame, text, (u+15, v), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    cv2.putText(frame, "PRESS SPACE TO PICK", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

        cv2.imshow("HCR5 Vision Feed", frame)
        
        # --- SPACEBAR LOGIC ---
        key = cv2.waitKey(1) & 0xFF
        if key == ord(' '):  # SPACEBAR pressed
            if self.latest_world_coords:
                self.trigger_move()
            else:
                self.get_logger().warn("No object detected to pick!")
        elif key == ord('q'):
            rclpy.shutdown()

    def trigger_move(self):
        msg = Point()
        msg.x, msg.y = self.latest_world_coords
        msg.z = 0.15  # Default Hover Z height
        self.coord_pub.publish(msg)
        self.get_logger().info(f"ACTION SENT: Move to X={msg.x:.3f}, Y={msg.y:.3f}")

def main():
    rclpy.init()
    node = HCR5Picker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()