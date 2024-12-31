import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

class controlRobot(Node):
    def __init__(self):
        super().__init__("controlRobot")
        self.create_subscription(Twist,"/cmd_vel",self.cmdCallback,10)
        self.ser = serial.Serial(self.port, self.baud_rate, timeout=1)
        self.get_logger().info(f"Connected to {self.port} at {self.baud_rate} baud rate.")

    def cmdCallback(self,msg):
        self.get_logger().info(f"x: {msg.linear.x},{msg.angular.z}")
        self.send_floats_to_arduino(msg.linear.x,msg.angular.z)

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