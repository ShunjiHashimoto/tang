#!/usr/bin/env python
# -*- coding: utf-8 -*-

## teleoperation_Cubase!　##

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import time
import RPi.GPIO as GPIO


R = 12
L = 13
DUTY = 50.0
ENABLE_r = 17
ENABLE_l = 18
top = 1000
bottom = 50
GPIO.setmode(GPIO.BCM)
GPIO.setup(R, GPIO.OUT)
GPIO.setup(L, GPIO.OUT)
GPIO.setup(ENABLE_r, GPIO.OUT)
GPIO.setup(ENABLE_l, GPIO.OUT)
GPIO.output(ENABLE_r, GPIO.LOW)
GPIO.output(ENABLE_l, GPIO.LOW)

p_r = GPIO.PWM(R, bottom)
p_l = GPIO.PWM(L, bottom)


p_r.start(0)
p_l.start(0)


def joy_listener():
    # start node
    rospy.init_node("joy_twist", anonymous=True)

    # subscribe to joystick messages on topic "joy"
    rospy.Subscriber("joy", Joy, joy_callback, queue_size=1)

    # keep node alive until stopped
    rospy.spin()

# called when joy message is received


def joy_callback(joy_msg):
    #value_0 = int(joy_msg.axes[0]*100)
    value_1 = int(joy_msg.axes[1] * 100)
    #value_3 = int(joy_msg.axes[3]*100)
    value_2 = int(joy_msg.axes[4] * 100)

    time.sleep(0.1)

    if value_1 == -0:
        value_1 = 0

    if value_2 == -0:
        value_2 = 0

    if value_1 > 0 and value_2 > 0:
        GPIO.output(ENABLE_r, GPIO.LOW)
        GPIO.output(ENABLE_l, GPIO.LOW)
        p_r.ChangeDutyCycle(value_1)
        p_l.ChangeDutyCycle(value_2)

    elif value_1 < 0 and value_2 < 0:
        GPIO.output(ENABLE_r, GPIO.HIGH)
        GPIO.output(ENABLE_l, GPIO.HIGH)
        p_r.ChangeDutyCycle(-(value_1))
        p_l.ChangeDutyCycle(-(value_2))

    else:
        p_r.stop()
        p_l.stop()
        p_r.start(0)
        p_l.start(0)


if __name__ == '__main__':
    try:
        joy_listener()
    except rospy.ROSInterruptException:
        pass

  # p_r.stop()
  # p_l.stop()

GPIO.cleanup()
