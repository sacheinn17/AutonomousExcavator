from gpiozero import AngularServo
from time import sleep

servo1 = AngularServo(2, min_angle=0, max_angle=180, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000)
servo2 = AngularServo(3, min_angle=0, max_angle=180, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000)
servo3 = AngularServo(4, min_angle=0, max_angle=180, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000)

def set_angle(angle,servo):
    servo.angle = angle
    sleep(1)
