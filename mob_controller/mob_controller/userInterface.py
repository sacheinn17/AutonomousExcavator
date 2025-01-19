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
from std_msgs.msg import Float32MultiArray
import streamlit as st
from geometry_msgs.msg import Twist

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

        self.lv = 0
        self.av = 0
        self.cmd_msg = Twist()
        self.broadcaster = TransformBroadcaster(self)
        self.ballState = self.create_publisher(Bool,"/ballState",10)

    def image_callback_(self,msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Display the image
        cv2.imshow('Received Image', frame)
        cv2.waitKey(1)

    def publish_velocity(self,dir):
        pass

def main():
    rclpy.init()
    try:
        node = ImageSubscriber()

        st.title("Robot Velocity Control with Streamlit")

        # Display current velocities
        st.write(f"### Current Velocities")
        st.write(f"Linear Velocity: {node.lv:.2f}")
        st.write(f"Angular Velocity: {node.av:.2f}")

        # Video Stream Section
        st.write("### Video Stream")
        cap = cv2.VideoCapture(0)
        frame_placeholder = st.empty()

        # Velocity Adjustment Buttons
        st.write("### Adjust Velocities")
        if st.button("Increase Linear Velocity"):
            node.lv += 0.1
        if st.button("Decrease Linear Velocity"):
            node.lv -= 0.1
        if st.button("Increase Angular Velocity"):
            node.av += 0.1
        if st.button("Decrease Angular Velocity"):
            node.av -= 0.1

        # Publish Velocity Commands
        st.write("### Publish Commands")
        if st.button("Move Forward"):
            node.lv = 0.5
            node.av = 0.0
            node.publish_velocity()

        if st.button("Move Backward"):
            node.lv = -0.5
            node.av = 0.0
            node.publish_velocity()

        if st.button("Turn Left"):
            node.lv = 0.0
            node.av = 0.5
            node.publish_velocity()

        if st.button("Turn Right"):
            node.lv = 0.0
            node.av = -0.5
            node.publish_velocity()

        rclpy.spin(node)
    # Video Stream Loop
        try:
            while cap.isOpened():
                ret, frame = cap.read()
                if not ret:
                    st.error("Failed to capture video. Exiting...")
                    break

                # Convert BGR to RGB for Streamlit
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                frame_placeholder.image(frame_rgb, channels="RGB")

                # Update displayed velocities dynamically
                st.experimental_rerun()

        except Exception as e:
            st.error(f"An error occurred: {e}")
        finally:
            cap.release()
    except KeyboardInterrupt as e:
        node.destroy_node()

    rclpy.shutdown()