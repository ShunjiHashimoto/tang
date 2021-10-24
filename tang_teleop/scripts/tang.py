#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time
import RPi.GPIO as GPIO
from sensor_msgs.msg import Joy
from tang_detection.msg import Command

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
BTN_Y = 0x0001
BTN_A = 0x0002
AXS_MAX = 1.0
AXS_OFF = 0.0

class TangController():
    def __init__(self):
        self.cmd = Command()
        self.btn = self.joy_l = self.joy_r = 0
        self.main = 1
        self.ref_pos = 350
        self.max_area = rospy.get_param("/tang_teleop/max_area")
        self.speed = rospy.get_param("/tang_teleop/speed")
        self.p_gain = 0.0027 * self.speed
        self.command = 0

        # subscribe to motor messages on topic "msg_topic"
        self.cmd_sub = rospy.Subscriber('tang_cmd', Command, self.cmd_Callback, queue_size=1)
   
        # subscribe to joystick messages on topic "joy"
        self.joy_sub = rospy.Subscriber("joy", Joy, self.joyCallback, queue_size=1)
        
    def modeChange(self):
        if self.main == 0:
            ### old_teleop start 
            motor_l = self.joy_l
            motor_r = self.joy_r
            
            time.sleep(0.1)
            
            if motor_l > 0 and motor_r > 0:
                GPIO.output(ENABLE_r, GPIO.LOW)
                GPIO.output(ENABLE_l, GPIO.LOW)
                p_r.ChangeDutyCycle(motor_l)
                p_l.ChangeDutyCycle(motor_r)
                rospy.loginfo("Go! | motor_l : %d | motor_r: %d", motor_l, motor_r)
            
            elif motor_l < 0 and motor_r < 0:
                GPIO.output(ENABLE_r, GPIO.HIGH)
                GPIO.output(ENABLE_l, GPIO.HIGH)
                p_r.ChangeDutyCycle(-(motor_l))
                p_l.ChangeDutyCycle(-(motor_r))
                rospy.loginfo("Back! | motor_l : %d | motor_r: %d", motor_l, motor_r)
            
            else:
                rospy.loginfo("Stop! | motor_l : %d | motor_r: %d",
                              motor_l, motor_r)
                p_r.stop()
                p_l.stop()
                p_r.start(0)
                p_l.start(0)

        else:
            motor_r = motor_l = self.speed
            if self.cmd.max_area == 0: return
            if (self.cmd.max_area >= self.max_area or self.cmd.is_human == 0):
                rospy.logwarn("Stop")
                p_r.stop()
                p_l.stop()
                p_r.start(0)
                p_l.start(0)
                return
            elif (self.command < 0):
                motor_r += self.command
                rospy.loginfo("Turn Left!")
            elif (self.command >= 0):
                motor_l -= self.command
                rospy.loginfo("Turn Right!")
            # rospy.logwarn(self.cmd.max_area)
            GPIO.output(ENABLE_r, GPIO.LOW)
            GPIO.output(ENABLE_l, GPIO.LOW)
            p_r.ChangeDutyCycle(motor_l)
            p_l.ChangeDutyCycle(motor_r)

    def p_control(self, cur_pos):
        """
        @fn p_control()
        @details P制御
        """
        return self.p_gain * (self.ref_pos - cur_pos)
    
    def cmd_Callback(self, msg):
        # 人の位置とサイズを得る
        self.cmd = msg
        self.command = self.p_control(self.cmd.pos)
        if (abs(self.command) > self.speed):
            self.command = 0
        # rospy.logwarn("Command: %lf", self.command)
        
    def joyCallback(self, joy_msg):
        newbtn = 0
        
        if(joy_msg.buttons[6]):
            newbtn |= BTN_BACK
        elif(joy_msg.buttons[0]):
            newbtn |= BTN_A
        elif(joy_msg.buttons[3]):
            newbtn |= BTN_Y
        
        joy_l = joy_msg.axes[1]
        if(joy_l <= -AXS_OFF):
            joy_l += AXS_OFF
        elif(joy_l >= AXS_OFF):
            joy_l -= AXS_OFF
        else:
            joy_l = 0
            
        joy_r = joy_msg.axes[4]
        if(joy_r <= -AXS_OFF):
            joy_r += AXS_OFF
        elif(joy_r >= AXS_OFF):
            joy_r -= AXS_OFF
        else:
            joy_r = 0
        
        self.joy_l = int(self.speed * joy_l / (AXS_MAX - AXS_OFF))
        self.joy_r = int(self.speed * joy_r / (AXS_MAX - AXS_OFF))
            
        push = ((~self.btn) & newbtn)
        self.btn = newbtn
        if(push & BTN_BACK):
            self.main = (self.main + 1)%2
        elif(push & BTN_Y):
            self.speed += 10
        elif(push & BTN_A):
            self.speed -= 10
            
        if(self.speed > 100):
            self.speed = 100
        elif(self.speed < 10):
            self.speed = 10

def main():
    # start node
    rospy.init_node("cubase", anonymous=True)
    instance = TangController()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        instance.modeChange()
        rate.sleep()
    rospy.spin()
                
if __name__ == '__main__':
    main()
    GPIO.cleanup()
