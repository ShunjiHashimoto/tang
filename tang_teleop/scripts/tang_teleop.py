#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import time
from sensor_msgs.msg import Joy

class TangTeleop():
    def __init__(self):
        self.btn = 0
        self.main = 0
        self.joy_l = 0
        self.joy_r = 0
        self.percent = 50
        self.BTN_BACK = 0x0100
        self.BTN_Y = 0x0001
        self.BTN_A = 0x0002
        self.AXS_MAX = 1.0
        self.AXS_OFF = 0.0
        self.joy_sub = rospy.Subscriber("joy", Joy, self.joyCallback, queue_size=1)
        
    def teleop(self):
        if(self.joy_r == -0):
            self.joy_r = 0
        if(self.joy_l == -0):
            self.joy_l = 0

        motor_l = self.joy_l
        motor_r = self.joy_l
        max = self.percent
        ## 右回り
        if(self.joy_r < 0):
            if(self.joy_l > 0):
                motor_r  += (self.joy_l * self.joy_r)/max
            else:
                motor_r -= self.joy_r
        ## 左回り
        elif(self.joy_r > 0):
            if(self.joy_l > 0):
                motor_l -= (self.joy_r * self.joy_l) / max 
            else:
                motor_l += self.joy_r 
        else:
            motor_l = self.joy_l
            motor_r = self.joy_l

        time.sleep(0.1)
        return motor_l, motor_r
        
    def joyCallback(self, joy_msg):
        newbtn = 0
        
        if(joy_msg.buttons[6]):
            newbtn |= self.BTN_BACK
        elif(joy_msg.buttons[0]):
            newbtn |= self.BTN_A
        elif(joy_msg.buttons[3]):
            newbtn |= self.BTN_Y
        
        joy_l = joy_msg.axes[1]
        if(joy_l <= -self.AXS_OFF):
            joy_l += self.AXS_OFF
        elif(joy_l >= self.AXS_OFF):
            joy_l -= self.AXS_OFF
        else:
            joy_l = 0
            
        joy_r = joy_msg.axes[3]
        if(joy_r <= -self.AXS_OFF):
            joy_r += self.AXS_OFF
        elif(joy_r >= self.AXS_OFF):
            joy_r -= self.AXS_OFF
        else:
            joy_r = 0
        
        self.joy_l = int(self.percent * joy_l / (self.AXS_MAX - self.AXS_OFF))
        self.joy_r = int(self.percent * joy_r / (self.AXS_MAX - self.AXS_OFF))

        push = ((~self.btn) & newbtn)
        self.btn = newbtn
        if(push & self.BTN_BACK):
            self.main = (self.main + 1)%2
        elif(push & self.BTN_Y):
            self.percent += 10
        elif(push & self.BTN_A):
            self.percent -= 10
            
        if(self.percent > 100):
            self.percent = 100
        elif(self.percent < 10):
            self.percent = 10

def main():
    rospy.init_node("tang_teleop", anonymous=True)
    tang_teleop = TangTeleop()
    rate = rospy.Rate(40)
    while not rospy.is_shutdown():
        tang_teleop.teleop()
        rate.sleep()
    rospy.spin()
                
if __name__ == '__main__':
    main()
