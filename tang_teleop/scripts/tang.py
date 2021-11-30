#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import time
import RPi.GPIO as GPIO
from sensor_msgs.msg import Joy
from tang_detection.msg import Command
from std_msgs.msg import Int16
from tang_teleop.msg import Modechange

# modeを選択
GPIO.setmode(GPIO.BCM)

gpio_pin_r = 18
gpio_pin_l = 17
# デジタル出力ピンを設定, 回転方向を決められる
# DIG1 = 11(LEFT), DIG2 = 12(RIGHT)
GPIO.setup(gpio_pin_r, GPIO.OUT)
GPIO.setup(gpio_pin_l, GPIO.OUT)

output_pin_r = 12
output_pin_l = 13
# アナログ出力ピンを設定、output_pinを32,33に設定
# ANA1 = 32(LEFT), ANA2 = 33(RIGHT)
GPIO.setup(output_pin_r, GPIO.OUT)
GPIO.setup(output_pin_l, GPIO.OUT)
GPIO.output(output_pin_r, GPIO.LOW)
GPIO.output(output_pin_l, GPIO.LOW)
# PWMサイクルを50Hzに設定
p_r = GPIO.PWM(output_pin_r, 50)
p_l = GPIO.PWM(output_pin_l, 50)

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
        self.current_param = Modechange()
        self.main = 0
        self.ref_pos = 350
        self.max_area = rospy.get_param("/tang_teleop/max_area")
        self.max_area_red = rospy.get_param("/tang_teleop/max_area_red")
        self.speed = rospy.get_param("/tang_teleop/speed")
        self.p_gain = 0.0027 * self.speed
        self.command = 0

        # subscribe to motor messages on topic "tang_cmd", 追跡対象の位置と大きさ
        self.cmd_sub = rospy.Subscriber('tang_cmd', Command, self.cmd_callback, queue_size=1)
        # subscribe to joystick messages on topic "joy"
        self.joy_sub = rospy.Subscriber("joy", Joy, self.joy_callback, queue_size=1)
        # publisher, モードと距離の閾値、赤色検出の閾値をpub
        self.mode_pub = rospy.Publisher('current_param', Modechange, queue_size=1)

    def mode_change(self):
        if self.main == 0:
            motor_l = self.joy_l
            motor_r = self.joy_r
            print(motor_r, motor_l)
            time.sleep(0.1)
            if motor_l >= 0 and motor_r >= 0:
                GPIO.output(gpio_pin_r, GPIO.HIGH)
                GPIO.output(gpio_pin_l, GPIO.HIGH)
                p_r.ChangeDutyCycle(motor_r)
                p_l.ChangeDutyCycle(motor_l)
                # rospy.loginfo("Go! | motor_l : %d | motor_r: %d", motor_l, motor_r)
            elif motor_l < 0 and motor_r < 0:
                GPIO.output(gpio_pin_r, GPIO.LOW)
                GPIO.output(gpio_pin_l, GPIO.LOW)
                p_r.ChangeDutyCycle(-(motor_r))
                p_l.ChangeDutyCycle(-(motor_l))
                # rospy.loginfo("Back! | motor_l : %d | motor_r: %d", motor_l, motor_r)
            else:
                pass
        
        elif self.main == 1:
            motor_r = motor_l = self.speed
            if self.cmd.max_area == 0: return
            if (self.cmd.max_area >= self.max_area or self.cmd.is_human == 0):
                rospy.logwarn("Stop")
                motor_r = motor_l = 0
            elif (self.command < 0):
                motor_r += self.command
                rospy.loginfo("AN2, right, 32pin, motor_r is up | Turn Left!")
            elif (self.command >= 0):
                motor_l -= self.command
                rospy.loginfo("AN1, left, 33pin, motor_l is up | Turn Right!")
            # rospy.logwarn(self.cmd.max_area)
            GPIO.output(gpio_pin_r, GPIO.HIGH)
            GPIO.output(gpio_pin_l, GPIO.HIGH)
            p_r.ChangeDutyCycle(motor_r)
            p_l.ChangeDutyCycle(motor_l)
            rospy.loginfo("motor_l %lf, motor_r %lf", motor_l, motor_r)
    
        elif self.main == 2:
            motor_r = motor_l = self.speed
            if self.cmd.max_area == 0: return
            if (self.cmd.max_area >= self.max_area_red or self.cmd.max_area < 60):
                rospy.logwarn("Stop")
                motor_r = motor_l = 0
            elif (self.command < 0):
                motor_r += self.command
                rospy.loginfo("AN2, right, 32pin, motor_r is up | Turn Left!")
            elif (self.command >= 0):
                motor_l -= self.command
                rospy.loginfo("AN1, left, 33pin, motor_l is up | Turn Right!")
            GPIO.output(gpio_pin_r, GPIO.HIGH)
            GPIO.output(gpio_pin_l, GPIO.HIGH)
            p_r.ChangeDutyCycle(motor_r)
            p_l.ChangeDutyCycle(motor_l)
            rospy.loginfo("motor_l %lf, motor_r %lf", motor_l, motor_r)

        
    def p_control(self, cur_pos):
        """
        @fn p_control()
        @details P制御
        """
        return self.p_gain * (self.ref_pos - cur_pos)
    
    def cmd_callback(self, msg):
        # 人の位置とサイズを得る
        self.cmd = msg
        self.command = self.p_control(self.cmd.pos)
        if (abs(self.command) > self.speed):
            self.command = 0
        # rospy.logwarn("Command: %lf", self.command)
        
    def joy_callback(self, joy_msg):
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
            self.main = (self.main + 1)%4
        elif(push & BTN_Y):
            self.speed += 10
        elif(push & BTN_A):
            self.speed -= 10
            
        if(self.speed > 100):
            self.speed = 100
        elif(self.speed < 10):
            self.speed = 10

        self.current_param.current_mode = self.main
        self.mode_pub.publish(self.current_param)	
        
def main():
    # start node
    rospy.init_node("tang_bringup", anonymous=True)
    instance = TangController()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        instance.mode_change()
        rate.sleep()
    rospy.spin()
                
if __name__ == '__main__':
    main()
    GPIO.cleanup()
