from rclpy.node import Node
import rclpy
import time
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle,GoalStatus
from custom_interfaces.action import Move

class action_client(Node):
    def __init__(self):
        super().__init__("action_client")

        self.client = ActionClient(self,Move,"move_robot")
        self.send_goal()

    def send_goal(self):
        self.get_logger().info("Sending goal")
        self.client.wait_for_server()
        goal = Move.Goal()
        goal.lin_x = 0.5
        goal.ang_z = 1.0

        self.client.send_goal_async(goal,feedback_callback=self.feedback_callback).add_done_callback(
            self.goal_done_callback)
        

    def feedback_callback(self,feedback_msg):
        self.get_logger().info(f"Feedback received: {feedback_msg.feedback.running}")

    def goal_done_callback(self,future):
        self.goal_accepted:ClientGoalHandle = future.result()

        if self.goal_accepted.accepted:
            self.get_logger().info("Goal accepted")
            self.goal_accepted.get_result_async().add_done_callback(self.get_result_callback)

    def get_result_callback(self,future):
        result = future.result().result
        self.get_logger().info(f"Result {result.pos_x} {result.pos_y}")

    def feedback_callback(self,feedback_msg):
        self.get_logger().info(f"Feedback received: {feedback_msg.feedback.running}")

    def cancel_goal(self):
        self.get_logger().info("Cancelling goal")
        self.goal_accepted.cancel_goal_async()

def main(args=None):
    rclpy.init(args=args)
    try:
        node = action_client()
        node.send_goal()
        rclpy.spin(node)
    except KeyboardInterrupt as e:
        node.cancel_goal()
        time.sleep(0.5)
        rclpy.shutdown()