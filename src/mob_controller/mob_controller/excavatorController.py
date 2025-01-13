import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import sys
import select
import termios
import tty
import time

class TeleopTwistKeyboard(Node):
    def __init__(self):
        super().__init__('Excavator_controler')
        self.twist_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.array_pub = self.create_publisher(Float32MultiArray, 'float_array_mob', 10)

        self.speed = 0.8
        self.turn = 1.0
        self.exec_profile = [[]]
        self.dump_profile = [[]]
        self.reset_profile = [[]]
        self.settings = termios.tcgetattr(sys.stdin)
        self.init_message()

    def init_message(self):
        self.get_logger().info("Use WASD to control movement, QE for rotation")
        self.get_logger().info("Press JKL for publishing float array")
        self.get_logger().info("Press CTRL+C to quit")

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        try:
            twist = Twist()
            array_msg = Float32MultiArray()
            array_msg.data = [0.0,0.0,0.0,20.0]
            while rclpy.ok():
                key = self.get_key()
                if key in ['w', 'a', 's', 'd','z','q','e']:
                    if key == 'a':
                        twist.linear.x = self.speed
                        twist.angular.z = 0.0
                    elif key == 'd':
                        twist.linear.x = -self.speed
                        twist.angular.z = 0.0
                    elif key == 's':
                        twist.linear.x = 0.0
                        twist.angular.z = self.turn
                    elif key == 'w':
                        twist.linear.x = 0.0
                        twist.angular.z = -self.turn
                    elif key == 'z':
                        twist.angular.z = 0.0
                        twist.linear.x = 0.0
                    elif key == 'q':
                        twist.linear.x = self.speed
                        twist.angular.z = -self.turn
                    elif key == 'e':
                        twist.linear.x = -self.speed
                        twist.angular.z = self.turn
                    self.twist_pub.publish(twist)

                elif key in ['j', 'k', 'l','u','i','o','y','h','m']:
                    if key == 'h':
                            array_msg.data[0]-=1.0 if array_msg.data[3] >0.0 else 0.0
                    elif key == 'j':
                            array_msg.data[1]-=1.0 if array_msg.data[3] >0.0 else 0.0
                    elif key == 'k':
                            array_msg.data[2]-=1.0 if array_msg.data[3] >0.0 else 0.0
                    elif key == 'l':
                            array_msg.data[3]-=1.0 if array_msg.data[3] >0.0 else 0.0
                    elif key == 'y':
                            array_msg.data[0]+=1.0 if array_msg.data[0] <180.0 else 0.0
                    elif key == 'u':
                            array_msg.data[1]+=1.0 if array_msg.data[1] <180.0 else 0.0
                    elif key == 'i':
                            array_msg.data[2]+=1.0 if array_msg.data[2] <180.0 else 0.0
                    elif key == 'o':
                            array_msg.data[3]+=1.0 if array_msg.data[3] <180.0 else 0.0
                    elif key == 'm':
                        array_msg.data[0]= 0.0
                        array_msg.data[1]= 0.0
                        array_msg.data[2]= 0.0
                    self.array_pub.publish(array_msg)
                    self.get_logger().info(f"Published Float32MultiArray: {array_msg.data}")
                elif key in ['v','b','n']:
                    if key == 'v':
                        for i in self.exec_profile:
                            array_msg.data = i
                            self.array_pub.publish(array_msg)
                            self.get_logger().info(f"Executing excavation profile {i} ...")
                            time.sleep(0.2)
                    elif key == 'b':
                        for i in self.dump_profile:
                            array_msg.data = i
                            self.array_pub.publish(array_msg)
                            self.get_logger().info(f"Executing dump profile {i} ...")
                            time.sleep(0.2)
                    elif key == 'n':
                        for i in self.reset_profile:
                            self.array_pub.publish(array_msg)
                            self.get_logger().info(f"Executing reset profile {i} ...")
                            time.sleep(0.2)      
                elif key == '\x03':  # CTRL+C
                    break
        finally:
            twist = Twist()
            self.twist_pub.publish(twist)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    teleop_twist_keyboard = TeleopTwistKeyboard()
    teleop_twist_keyboard.run()
    teleop_twist_keyboard.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
