import cv2
import rclpy
from rclpy.node import Node
import numpy as np
from cv_bridge import CvBridge
from tf2_geometry_msgs import TransformStamped
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import os
def detect_yellow_box(image):

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 255])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    d = -1
    x = -1
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        center_x = x + w // 2
        center_y = y + h // 2
        d = 634.0/(w)
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.circle(image, (center_x, center_y), 5, (0, 0, 255), -1)
        cv2.putText(image,f"Area is {w*h}, x : {round(d,2)} Y : {round((x-250)//1000.0,2)} ",(10,50),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,255),2)
        print(f"Yellow box position: x={center_x}, y={center_y}")
    return (image,d,x)

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__("robo")
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        # os.environ.pop("QT_QPA_PLATFORM_PLUGIN_PATH")
        # self.img = self.create_subscription(Image,"/camera",self.image_callback_,qos)
        self.img = self.create_subscription(CompressedImage,"/camera",self.image_callback_,qos)

        self.broadcaster = TransformBroadcaster(self)
        self.ballState = self.create_publisher(Bool,"/ballState",10)
    # def image_callback_(self,msg):
    #         self.image = CvBridge().imgmsg_to_cv2(msg)
    #         self.image = cv2.cvtColor(self.image, cv2.COLOR_RGB2BGR)
    #         image,d,x= detect_yellow_box(self.image)
    #         b = Bool()
    #         if d>=0:
    #             #  self.objPos.publish(arr)
    #             obj = TransformStamped()
    #             obj.header.frame_id = "camera_link"
    #             obj.child_frame_id = "object"
    #             obj.transform.translation.x = d
    #             obj.transform.translation.y = (x-250)//1000.0
    #             obj.transform.translation.z = 0.0
    #             obj.header.stamp = self.get_clock().now().to_msg()
    #             self.broadcaster.sendTransform(obj)
    #             b.data = True
    #         else:
    #             b.data = False
    #         self.ballState.publish(b)
    #         cv2.imshow("image",image)
    #         cv2.waitKey(1)

    def image_callback_(self,msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Display the image
        cv2.imshow('Received Image', frame)
        cv2.waitKey(1)  # Needed to display images properly in a loop
        # self.get_logger().error(f"Failed to decode image: {e}")


def main():
    rclpy.init()

    try:
        node = ImageSubscriber()
        rclpy.spin(node)
    except KeyBoardInterupt as e:
        
    node.destroy_node()

    rclpy.shutdown()