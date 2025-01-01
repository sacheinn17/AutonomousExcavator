import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

import serial.tools.list_ports
from sensor_msgs.msg import JointState

class controlRobot(Node):
    def __init__(self):
        super().__init__("controlRobot")

        self.wheel_radius = 0
        self.wheel_dist = 0
        self.vel = Twist()

        self.ports = serial.tools.list_ports.comports()
        self.t = []
        for port, desc, hwid in sorted(self.ports):
                print("{}: {} [{}]".format(port, desc, hwid))
                self.t.append(port)
        print(self.ports)
        self.port = self.t[0]
        self.baud_rate = 9600

        self.p_time = self.get_clock.now()
        self.wheel_pos = [0.0,0.0,0.0,0.0]

        self.joint_state_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_subscription(Twist,"/cmd_vel",self.cmdCallback,10)
        self.ser = serial.Serial(self.port, self.baud_rate, timeout=1)
        self.get_logger().info(f"Connected to {self.port} at {self.baud_rate} baud rate.")
        self.timer = self.create_timer(0.1,self.timerCallback)

    def cmdCallback(self,msg:Twist):
        self.vel = msg
        self.get_logger().info(f"x: {msg.linear.x},{msg.angular.z}")
        self.wl = (2*msg.linear.x - msg.angular.z*self.wheel_dist)/(2*self.wheel_radius)
        self.wr = (2*msg.linear.x + msg.angular.z*self.wheel_dist)/(2*self.wheel_radius)
        self.send_floats_to_arduino(self.wl,self.wr)

    def timerCallback(self):
        self.joint_state = JointState()
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.joint_state.name = ['base_left_back_joint', 'base_right_back_joint','base_left_front_joint', 'base_right_front_joint']
        self.wheel_pos[0] += self.wl*0.1
        self.wheel_pos[1] += self.wr*0.1
        self.wheel_pos[2] += self.wl*0.1
        self.wheel_pos[3] += self.wr*0.1
        self.joint_state.position = self.wheel_pos
        self.joint_state.velocity = [self.wl,self.wr,self.wl,self.wr]
        self.joint_state.effort = [0.0,0.0,0.0,0.0]
        self.joint_state_pub.publish(self.joint_state)

    def send_floats_to_arduino(self,float1,float2):
        try:          
            data = f"{float1},{float2}\n"
            print(f"Sending data: {data.strip()}")
            self.ser.write(data.encode())
            response = self.ser.readline().decode('utf-8').strip()
            if response:
                print(f"Arduino response: {response}")
                
        except ValueError:
            self.get_logger().warn("Invalid data provided,only float values are accpted")
        
        except KeyboardInterrupt:
            print("\nTerminating the program.")
            
            # Close the serial connection
            self.ser.close()
            print("Connection closed.")
            
        except Exception as e:
            self.get_logger().error(f"Error: {e}")