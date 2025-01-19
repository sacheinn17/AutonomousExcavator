from gpiozero import AngularServo
from time import sleep

servos = [AngularServo(2, min_angle=0, max_angle=180, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000),
AngularServo(3, min_angle=0, max_angle=180, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000),
AngularServo(4, min_angle=0, max_angle=180, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000),
AngularServo(4, min_angle=0, max_angle=180, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000)]

def set_angle(angles):
    for i in len(angles):
        servos[i].angle = angles[i]
    sleep(0.2)
