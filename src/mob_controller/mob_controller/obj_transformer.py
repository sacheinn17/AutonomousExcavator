import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from tf2_geometry_msgs import TransformStamped
from std_msgs.msg import Float32MultiArray
class transformer(Node):
    def __init__(self):
        super().__init__("transformer")
        self.broadcaster = TransformBroadcaster(self)
        self.objpos = self.create_subscription(Float32MultiArray,"/objpos",self.obj_pos_callback,10)


    def obj_pos_callback(self,msg):
        obj = TransformStamped()
        obj.header.frame_id = "camera_link"
        obj.child_frame_id = "object"
        obj.transform.translation.x = msg.data[0]
        obj.transform.translation.y = msg.data[1]
        obj.transform.translation.z = msg.data[2]
        obj.header.stamp = self.get_clock().now().to_msg()
        self.broadcaster.sendTransform(obj)

def main():
    rclpy.init()

    node = transformer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

