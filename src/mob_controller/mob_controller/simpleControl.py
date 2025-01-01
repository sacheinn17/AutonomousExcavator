from rclpy.node import Node
import rclpy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import math
from tf2_ros import TransformListener,Buffer
from tf2_ros import Duration,Time,Clock
from std_msgs.msg import Bool
import math
import time
class moveRobot(Node):
    def __init__(self):
        super().__init__("simple_controller")

        self.pose_x = 0
        self.pose_y = 0
        self.orientation_z = 0
        self.imuVal = Imu()
        self.imu = self.create_subscription(Imu,"/imu",self.imuCallBack,10)
        self.cmdVel = self.create_publisher(Twist,"/cmd_vel",10)
        self.msg = Twist()
        self.timer = self.create_timer(0.1,self.timerCallBack)
        self.prevTime = self.get_clock().now()
        self.dx = 0
        self.ballState = True

        self.buffer = Buffer()
        self.tf_listener = TransformListener(self.buffer,self)
        self.ballState = self.create_subscription(Bool,"/ballState",self.ballStateCallBack,10)
    def ballStateCallBack(self,msg):
        self.ballState = msg.data
        # print(msg.data)
    def imuCallBack(self,msg:Imu):
        self.imuVal = msg

        acc_x=self.imuVal.linear_acceleration.x
        acc_y=self.imuVal.linear_acceleration.y

        # print(acc_x,acc_y)
    def randomAccel(self):
        acc_x=self.imuVal.linear_acceleration.x
        acc_y=self.imuVal.angular_velocity.z

        self.currentTime = self.get_clock().now()
        dt = (self.currentTime-self.prevTime).nanoseconds/1000000000.0
        self.orientation_z += acc_y*dt
        self.pose_x += (acc_x*(dt)*math.cos(self.orientation_z))*dt
        self.pose_y += acc_x*(dt**2)*math.sin(self.orientation_z)
        # print(self.pose_x,self.pose_y,self.orientation_z)
        if acc_x < 2:
            self.msg.linear.x += 0.01
        else:
            self.msg.linear.x = 0.0
        if acc_y < 2:
            self.msg.angular.z += 0.01
        else:
            self.msg.angular.z = 0.0
        self.cmdVel.publish(self.msg)

    def moveTowardsTarget(self):
        try:
            object = self.buffer.lookup_transform("object","base_link",Time().to_msg(),Duration(seconds=1.0))
            msg = Twist()

            if -1*object.transform.translation.x > 10.0:
                msg.linear.x = 1.0
                msg.angular.z = -10*math.atan(object.transform.translation.y/object.transform.translation.x)
            else:
                msg.angular.z = 0.0
                msg.linear.x = 0.0

            print(object.transform.translation.y)
            self.cmdVel.publish(msg)
        except Exception as e:
            print("Exception ",e)

    def timerCallBack(self):
        # self.randomAccel()
        if self.ballState == True:
            # print("ball")
            self.moveTowardsTarget()
        else:
            print("searching")
            self.searchForBall()
    def searchForBall(self):
        self.msg = Twist()
        self.msg.linear.x = 0.0
        self.msg.angular.z = 5.0
        self.cmdVel.publish(self.msg)
        time.sleep(0.2)
    def close(self):
        print("Process stoping")
        stop = Twist()
        stop.linear.x = 0.0
        stop.angular.z = 0.0

        print("Stopping")

        self.cmdVel.publish(stop)

def main():
    rclpy.init()
    n = moveRobot()


    try:
        rclpy.spin(n)
    except KeyboardInterrupt as e:

        n.close()
        n.destroy_node()
        rclpy.shutdown()
