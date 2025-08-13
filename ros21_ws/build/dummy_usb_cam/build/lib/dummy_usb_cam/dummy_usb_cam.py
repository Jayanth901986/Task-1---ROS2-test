import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class DummyUSBCam(Node):
    def __init__(self):
        super().__init__('dummy_usb_cam')
        self.publisher_ = self.create_publisher(Image, '/image_raw', 10)
        self.bridge = CvBridge()

        # Hardcoded path to your image file
        file_path = '/home/jayanth/photos/test.jpg'
        self.publish_rate = 30.0  # Target FPS

        if not os.path.exists(file_path):
            self.get_logger().error(f"File not found: {file_path}")
            rclpy.shutdown()
            return

        # Detect if video or image
        if file_path.lower().endswith(('.mp4', '.avi', '.mov', '.mkv')):
            self.cap = cv2.VideoCapture(file_path)
            self.is_video = True
        else:
            self.image = cv2.imread(file_path)
            if self.image is None:
                self.get_logger().error("Could not load image file.")
                rclpy.shutdown()
                return
            self.is_video = False

    def publish_frame(self):
        if self.is_video:
            ret, frame = self.cap.read()
            if not ret:
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # loop video
                return
        else:
            frame = self.image.copy()

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DummyUSBCam()

    rate = node.create_rate(node.publish_rate)  # 30 Hz loop
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0)  # process callbacks if any
            node.publish_frame()
            rate.sleep()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
