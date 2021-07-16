#!/usr/bin/env python
# -*- coding: utf-8 -*-

## object_detection_Cubase & tele_operation!　##

import rospy
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import RPi.GPIO as GPIO
from std_msgs.msg import String

top = 1000
bottom = 50
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

BTN_BACK = 0x0100
AXS_MAX = 1.0
AXS_OFF = 0.0

class cubase():
    def __init__(self):
        self.message = "stop"
        self.btn = 0
        self.main = 0
        self.lud = 0
        self.rud = 0

        # subscribe to motor messages on topic "msg_topic"
        self._msg_sub = rospy.Subscriber("/msg_topic", String, self.msg_callback, queue_size=1)
        
        # subscribe to joystick messages on topic "joy"
        self._joy_sub = rospy.Subscriber("joy", Joy, self.joy_callback, queue_size=1)
        
    def msg_listener(self):
        if self.main == 0:
            ### teleop start
            
            #value_0 = int(joy_msg.axes[0]*100)
            value_1 = self.lud
            #value_3 = int(joy_msg.axes[3]*100)
            value_2 = self.rud
            
            time.sleep(0.1)
            
            if value_1 > 0 and value_2 > 0:
                GPIO.output(ENABLE_r, GPIO.LOW)
                GPIO.output(ENABLE_l, GPIO.LOW)
                p_r.ChangeDutyCycle(value_1)
                p_l.ChangeDutyCycle(value_2)
                print("go:", value_1, value_2)
            
            elif value_1 < 0 and value_2 < 0:
                GPIO.output(ENABLE_r, GPIO.HIGH)
                GPIO.output(ENABLE_l, GPIO.HIGH)
                p_r.ChangeDutyCycle(-(value_1))
                p_l.ChangeDutyCycle(-(value_2))
                print("back:", value_1, value_2)
            
            else:
                print("stop:", value_1, value_2)
                p_r.stop()
                p_l.stop()
                p_r.start(0)
                p_l.start(0)
        else:
            if self.message == "go ahead":
                print("go ahead")
                value_1 = 80.0
                value_2 = 80.0
                GPIO.output(ENABLE_r, GPIO.LOW)
                GPIO.output(ENABLE_l, GPIO.LOW)
                p_r.ChangeDutyCycle(value_1)
                p_l.ChangeDutyCycle(value_2)
                
            elif self.message == "turn right":
                print("turn right")
                value_1 = 80.0
                value_2 = 10.0
                GPIO.output(ENABLE_r, GPIO.LOW)
                GPIO.output(ENABLE_l, GPIO.LOW)
                p_r.ChangeDutyCycle(value_1)
                p_l.ChangeDutyCycle(value_2)
                
            elif self.message == "turn left":
                print("turn left")
                value_1 = 10.0
                value_2 = 80.0
                GPIO.output(ENABLE_r, GPIO.LOW)
                GPIO.output(ENABLE_l, GPIO.LOW)
                p_r.ChangeDutyCycle(value_1)
                p_l.ChangeDutyCycle(value_2)
                
            elif self.message == "stop":
                print("stop")
                p_r.stop()
                p_l.stop()
                p_r.start(0)
                p_l.start(0)
            else:
                print("error")
                #time.sleep(0.1)
    
    def msg_callback(self, msg):
        self.message = msg.data
        
    def joy_callback(self, joy_msg):
        newbtn = 0
        
        if(joy_msg.buttons[6]):
            newbtn |= BTN_BACK
        
        ldu = joy_msg.axes[1]
        if(ldu <= -AXS_OFF):
            ldu += AXS_OFF
        elif(ldu >= AXS_OFF):
            ldu -= AXS_OFF
        else:
            ldu = 0
            
        rdu = joy_msg.axes[4]
        if(rdu <= -AXS_OFF):
            rdu += AXS_OFF
        elif(rdu >= AXS_OFF):
            rdu -= AXS_OFF
        else:
            rdu = 0
        
        self.lud = int(100 * ldu / (AXS_MAX - AXS_OFF))
        self.rud = int(100 * rdu / (AXS_MAX - AXS_OFF))
            
        push = ((~self.btn) & newbtn)
        self.btn = newbtn
        if(push & BTN_BACK):
            self.main = (self.main + 1)%2

def main():
    # start node
    rospy.init_node("cubase", anonymous=True)
        
    instance = cubase()
 
    # ratesleep
    rate = rospy.Rate(40)
    
    while not rospy.is_shutdown():
        instance.msg_listener()
        rate.sleep()
        
    # spin
    rospy.spin()
                
if __name__ == '__main__':
    main()
    GPIO.cleanup()

