import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool
from cv_bridge import CvBridge
import cv2

class ImageConversion(Node):
    def __init__(self):
        super().__init__('image_conversion')
        self.bridge = CvBridge()
        self.mode_grayscale = False

        self.declare_parameter('input_topic', '/image_raw')
        self.declare_parameter('output_topic', '/image_converted')

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value

        self.sub = self.create_subscription(Image, self.input_topic, self.callback, 10)
        self.pub = self.create_publisher(Image, self.output_topic, 10)
        self.srv = self.create_service(SetBool, 'change_mode', self.change_mode_cb)

    def change_mode_cb(self, request, response):
        self.mode_grayscale = request.data
        response.success = True
        response.message = 'Mode set to grayscale' if request.data else 'Mode set to color'
        self.get_logger().info(response.message)
        return response

    def callback(self, msg):
        cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        if self.mode_grayscale:
            cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
            msg_out = self.bridge.cv2_to_imgmsg(cv_img, "mono8")
        else:
            msg_out = msg
        self.pub.publish(msg_out)

def main():
    rclpy.init()
    node = ImageConversion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
