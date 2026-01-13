import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')
        
        # 1. Create a subscriber for the camera topic
        # Change 'camera1/image_raw' to whichever camera you want to view
        self.subscription = self.create_subscription(
            Image,
            'camera1/image_raw',
            self.listener_callback,
            10)
        
        # 2. Initialize the CvBridge
        self.bridge = CvBridge()

    def listener_callback(self, data):
        # 3. Convert ROS Image message to OpenCV format
        current_frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

        # 4. (Optional) Basic OpenCV Processing - Draw a circle in the center
        height, width = current_frame.shape[:2]
        cv2.circle(current_frame, (width//2, height//2), 10, (0, 0, 255), -1)

        # 5. Display the frame
        cv2.imshow("OpenCV Camera View", current_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    viewer = CameraViewer()
    rclpy.spin(viewer)
    viewer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()