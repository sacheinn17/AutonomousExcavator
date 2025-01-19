from rclpy.node import Node
import rclpy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from tf2_ros import TransformListener,Buffer
from rclpy.action import ActionServer,GoalResponse
from rclpy.action.server import ServerGoalHandle
from custom_interfaces.action import Move
import time
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup


class moveRobot(Node):
    def __init__(self):
        super().__init__("action_controller")

        self.imuVal = Imu()
        self.imu = self.create_subscription(Imu,"/imu",self.imuCallBack,10)
        self.cmdVel = self.create_publisher(Twist,"/cmd_vel",10)
        self.prevTime = self.get_clock().now()

        self.move_action_server = ActionServer(
            self,Move,
            "move_robot",
            goal_callback=self.move_goal_callback,
            execute_callback=self.move_execute_callback,
            cancel_callback=self.move_cancel_callback,
            callback_group= ReentrantCallbackGroup()
            )

        self.buffer = Buffer()
        self.tf_listener = TransformListener(self.buffer,self)

    def move_goal_callback(self,goal_request:Move.Goal):
        return GoalResponse.ACCEPT

    def move_execute_callback(self,goal_handle:ServerGoalHandle):
        self.move_cmd = Twist()
        #The required message is present in goal_handle.request
        self.move_cmd.linear.x = goal_handle.request.lin_x
        self.move_cmd.angular.z = goal_handle.request.ang_z
        if goal_handle.is_cancel_requested:
            self.move_cmd.linear.x = 0.0
            self.move_cmd.angular.z = 0.0
            self.get_logger().info("Goal cancelled ")
            self.cmdVel.publish(self.move_cmd)
            goal_handle.canceled()
            result.pos_x = 1.0
            result.pos_y = 2.0
            return result
                
        self.cmdVel.publish(self.move_cmd)

        feedback = Move.Feedback()
        feedback.running = True
        goal_handle.publish_feedback(feedback)
        time.sleep(10.0)
        goal_handle.succeed()
        result = Move.Result()
        result.pos_x = 1.0
        result.pos_y = 2.0
        return result

    def move_cancel_callback(self,goal_handle:ServerGoalHandle):
        self.move_cmd.linear.x = 0.0
        self.move_cmd.angular.z = 0.0
        self.get_logger().info("Goal cancelled ")
        self.cmdVel.publish(self.move_cmd)
        return GoalResponse.ACCEPT

    def imuCallBack(self,msg:Imu):
        self.imuVal = msg

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
        rclpy.spin(n,executor=MultiThreadedExecutor())
    except KeyboardInterrupt as e:
        n.close()
        n.destroy_node()
        rclpy.shutdown()
