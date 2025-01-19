import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
from std_msgs import Float32MultiArray
import serial.tools.list_ports
from sensor_msgs.msg import JointState
from servoController import set_angle

class robo_controller(Node):
    def __init__(self):
        super().__init__("robo_controller")

        self.wheel_radius = 0.0325
        self.wheel_dist = 0.2155
        self.vel = Twist()

        self.ports = serial.tools.list_ports.comports()
        self.t = []
        for port, desc, hwid in sorted(self.ports):
                print("{}: {} [{}]".format(port, desc, hwid))
                self.t.append(port)
        print(self.ports)
        self.port = self.t[0]
        self.baud_rate = 9600

        self.wheel_pos = [0.0,0.0,0.0,0.0]
        self.wl = 0.0
        self.wr = 0.0

        self.joint_state_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_subscription(Twist,"/cmd_vel",self.cmdCallback,10)
        self.create_subscription(Float32MultiArray, 'float_array_mob',self.excavatorCallback, 10)
        self.ser = serial.Serial(self.port, self.baud_rate, timeout=1)
        self.get_logger().info(f"Connected to {self.port} at {self.baud_rate} baud rate.")
        self.timer = self.create_timer(0.1,self.timerCallback)

    def cmdCallback(self,msg:Twist):
        self.vel = msg
        self.get_logger().info(f"x: {msg.linear.x},{msg.angular.z}")
        self.wl = (2*msg.linear.x + msg.angular.z*self.wheel_dist)/(2*self.wheel_radius)
        self.wr = (2*msg.linear.x - msg.angular.z*self.wheel_dist)/(2*self.wheel_radius)
        self.send_floats_to_arduino(1,[self.wl,self.wr])

    def excavatorCallback(self,msg):
        self.send_floats_to_arduino(2,list(msg.data))

    def timerCallback(self):
        self.joint_state = JointState()
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.joint_state.name = ['base_left_back_wheel_joint', 'base_right_back_wheel_joint','base_left_front_wheel_joint', 'base_right_front_wheel_joint']
        self.wheel_pos[0] += self.wl*0.1
        self.wheel_pos[1] += self.wr*0.1
        self.wheel_pos[2] += self.wl*0.1
        self.wheel_pos[3] += self.wr*0.1
        self.joint_state.position = self.wheel_pos
        self.joint_state.velocity = [self.wl,self.wr,self.wl,self.wr]
        self.joint_state.effort = [0.0,0.0,0.0,0.0]
        self.joint_state_pub.publish(self.joint_state)

    def send_floats_to_arduino(self,ID,values):
        try:
            data = f"{ID},"+",".join([str(i) for i in values]) + "\n"
            print(f"Sending data: {data.strip()}")
            self.ser.write(data.encode())
            response = self.ser.readline().decode('utf-8').strip()
            if response:
                print(f"Arduino response: {response}")
                
        except ValueError:
            self.get_logger().warn("Invalid data provided,only float values are accpted")
        
        except KeyboardInterrupt:
            print("\nTerminating the program.")
         
            self.ser.close()
            print("Connection closed.")
            
        except Exception as e:
            self.get_logger().error(f"Error: {e}")


def main():
    rclpy.init()
    node = robo_controller()
    rclpy.spin(node)
    rclpy.shutdown()