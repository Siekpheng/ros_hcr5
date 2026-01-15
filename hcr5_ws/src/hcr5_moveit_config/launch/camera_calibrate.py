import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

# Coordinates of the 4 table corners from your URDF (X, Y in meters)
# Based on your table size (0.75 x 0.985)
# Updated WORLD_PTS to a smaller visible rectangle on the table
# The order must match your clicking order!
# Order: Red (1), Green (2), Blue (3), Yellow (4)
WORLD_PTS = np.array([
    [0.4,  0.2], # Red
    [0.6,  0.2], # Green
    [0.6, -0.2], # Blue
    [0.4, -0.2]  # Yellow
], dtype="float32")

class Calibrator(Node):
    def __init__(self):
        super().__init__('calibrator')
        self.sub = self.create_subscription(Image, 'camera1/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()
        self.image_pts = []
        self.got_image = False
        self.img = None
        print("Waiting for image from Gazebo...")

    def image_callback(self, msg):
        if not self.got_image:
            self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.got_image = True
            self.run_calibration()

    def click_event(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.image_pts.append([x, y])
            cv2.circle(self.img, (x, y), 5, (0, 0, 255), -1)
            cv2.imshow("Calibrate", self.img)
            
            if len(self.image_pts) == 4:
                M = cv2.getPerspectiveTransform(np.array(self.image_pts, dtype="float32"), WORLD_PTS)
                print("\n--- Calibration Matrix (M) ---")
                print(M)
                np.save("table_calibration.npy", M)
                print("\nSaved as table_calibration.npy! You can close the window now.")

    def run_calibration(self):
        print("Image received! Click the 4 corners of the table in order:")
        print("1. Top-Left, 2. Top-Right, 3. Bottom-Right, 4. Bottom-Left")
        cv2.imshow("Calibrate", self.img)
        cv2.setMouseCallback("Calibrate", self.click_event)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        rclpy.shutdown()

def main():
    rclpy.init()
    node = Calibrator()
    rclpy.spin(node)

if __name__ == '__main__':
    main()