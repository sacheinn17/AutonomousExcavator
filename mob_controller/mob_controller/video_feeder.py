import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
#from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
class video_feeder(Node):
    def __init__(self):
        super().__init__("video_feeder")
        self.get_logger().info("Video feed started")
        self.image_capture = cv2.VideoCapture(0)
        self.msg = CompressedImage()
        qos = rclpy.qos.QoSProfile(reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL,depth=10)
        self.cameraPub = self.create_publisher(CompressedImage,"/camera",qos)
        self.timer = self.create_timer(0.1,self.timerCallback)

    def timerCallback(self):
        ret,frame = self.image_capture.read()
        if ret:
            frame = cv2.resize(frame, (480,360)) 
            self.msg.format="jpeg"
            _,img = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 40])
            self.msg.data = img.tobytes()    
            self.cameraPub.publish(self.msg)
            self.get_logger().info("Image published")

def main():
    rclpy.init()
    try:
        node = video_feeder()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()
