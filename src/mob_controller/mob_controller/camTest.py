import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class camTester(Node):
    def __init__(self):
        super().__init__("camTester")
        self.get_logger().info("Camera tester node created")
        self.image_capture = cv2.VideoCapture(-1)
        self.bridge = CvBridge()

        self.cameraPub = self.create_publisher(Image,"/camera",10)
        self.timer = self.create_timer(0.1,self.timerCallback)

    def timerCallback(self):
        ret,frame = self.image_capture.read()
        if ret:
            img = self.bridge.cv2_to_imgmsg(frame)
            self.cameraPub.publish(img)
            self.get_logger().info("Image published")

def main():
    rclpy.init()
    try:
        node = camTester()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()