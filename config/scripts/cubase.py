#!/usr/bin/env python
# -*- coding: utf-8 -*-

## object_detection_Cubase!　##

import rospy
from sensor_msgs.msg import Joy
import time
import RPi.GPIO as GPIO
from std_msgs.msg import String

R = 12
L = 13
ENABLE_r = 17
ENABLE_l = 18
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


def msg_listener():
    # start node
    rospy.init_node("cubase", anonymous=True)

    # subscribe to motor messages on topic "msg_topic"
    rospy.Subscriber("/msg_topic", String, msg_callback, queue_size=1)

    # keep node alive until stopped
    rospy.spin()


def msg_callback(msg):
    print(msg)
    if msg.data == "go ahead":
        # print("go ahead")
        value_1 = 60.0
        value_2 = 60.0
        GPIO.output(ENABLE_r, GPIO.LOW)
        GPIO.output(ENABLE_l, GPIO.LOW)
        p_r.ChangeDutyCycle(value_1)
        p_l.ChangeDutyCycle(value_2)

    elif msg.data == "turn right":
        # print("turn right")
        value_1 = 0.0
        value_2 = 80.0
        GPIO.output(ENABLE_r, GPIO.LOW)
        GPIO.output(ENABLE_l, GPIO.LOW)
        p_r.ChangeDutyCycle(value_1)
        p_l.ChangeDutyCycle(value_2)

    elif msg.data == "turn left":
        # print("turn left")
        value_1 = 80.0
        value_2 = 0.0
        GPIO.output(ENABLE_r, GPIO.LOW)
        GPIO.output(ENABLE_l, GPIO.LOW)
        p_r.ChangeDutyCycle(value_1)
        p_l.ChangeDutyCycle(value_2)

    elif msg.data == "stop":
        # print("stop")
        p_r.stop()
        p_l.stop()
        p_r.start(0)
        p_l.start(0)

    else:
        print("error")

    time.sleep(0.1)


if __name__ == '__main__':
    try:
        msg_listener()
    except rospy.ROSInterruptException:
        pass

GPIO.cleanup()
