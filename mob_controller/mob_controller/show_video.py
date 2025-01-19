import cv2
import rclpy
from rclpy.node import Node
import numpy as np
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class show_video(Node):
    def __init__(self):
        super().__init__("robo")
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        self.img = self.create_subscription(CompressedImage,"/camera",self.image_callback_,qos)
        self.broadcaster = TransformBroadcaster(self)
        self.ballState = self.create_publisher(Bool,"/ballState",10)

    def image_callback_(self,msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Display the image
        cv2.imshow('Received Image', frame)
        cv2.waitKey(1)  


def main():
    rclpy.init()

    node = show_video()
    rclpy.spin(node)
    node.destroy_node()

    rclpy.shutdown()