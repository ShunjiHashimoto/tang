#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import time
import RPi.GPIO as GPIO
from sensor_msgs.msg import Joy
from tang_msgs.msg import HumanInfo, Modechange
from geometry_msgs.msg import Twist

# modeを選択
GPIO.setmode(GPIO.BCM)

gpio_pin_r = 18
gpio_pin_l = 17
# デジタル出力ピンを設定, 回転方向を決められる
# DIG1 = 11(LEFT), DIG2 = 12(RIGHT)
GPIO.setup(gpio_pin_r, GPIO.OUT)
GPIO.setup(gpio_pin_l, GPIO.OUT)

output_pin_r = 13
output_pin_l = 12
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
        # HumanInfo.msg
        self.human_info = HumanInfo()
        self.cmdvel_from_imu = Twist()
        # Modechange.msg
        self.current_mode = Modechange()
        self.current_mode.realsense_depth_thresh = 4.0
        self.btn = self.joy_l = self.joy_r = 0
        self.main = 0
        self.ref_pos = 0.0
        self.speed = rospy.get_param("/tang_control/speed")
        self.prev_command = 0
        self.command_pwm = 0
        self.depth_min_thresh = 0.8
        self.dt = 0.1

        # subscribe to motor messages on topic "tang_cmd", 追跡対象の位置と大きさ
        self.human_info_sub = rospy.Subscriber('tang_cmd', Command, self.cmd_callback, queue_size=1)
        self.imu_sub = rospy.Subscriber('cmdvel_from_imu', Twist, self.imu_callback, queue_size=1)
        # subscribe to joystick messages on topic "joy"
        self.joy_sub = rospy.Subscriber("joy", Joy, self.joy_callback, queue_size=1)
        # publisher, モードと距離の閾値、赤色検出の閾値をpub
        self.mode_pub = rospy.Publisher('current_param', Modechange, queue_size=1)

    def mode_change(self):
        if self.main == 0:
            motor_l = self.joy_l
            motor_r = self.joy_r
            # print(motor_r, motor_l)
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
                p_l.ChangeDutyCycle(-(motor_l*1.1))
                # rospy.loginfo("Back! | motor_l : %d | motor_r: %d", motor_l, motor_r)
            return
        
        elif self.main == 1:
            # 速度を距離に従って減衰させる、1m20cm以内で減衰開始する
            if(self.human_info.human_point.x >= 1.2 or self.human_info.human_point.x < 0.0):
                command_depth = 1.0
            else:
                command_depth = self.human_info.human_point.x / 1.2
            motor_r = motor_l = self.speed * command_depth
            
            # コマンドの制御量を比例制御で決める
            self.command_pwm = self.p_control(self.human_info.human_point.y)

            if (abs(self.command_pwm) > motor_r and self.command_pwm < 0.0):
                self.command_pwm = -self.speed * command_depth
            elif (abs(self.command_pwm) > motor_r and self.command_pwm > 0.0):
                self.command_pwm = self.speed * command_depth
            # 80cm以内であれば止まる
            if self.human_info.human_point.x <= self.depth_min_thresh and self.human_info.human_point.x > 0.0:
                rospy.logwarn("Stop: %lf", self.human_info.human_point.x)
                motor_r = motor_l = 0
            elif (self.command_pwm < 0): # 右回り
                motor_r -= self.command_pwm/10
                motor_r = motor_r*1.1
                motor_l += self.command_pwm 
                rospy.loginfo("motor_l %lf, motor_r %lf , | Turn Right", motor_l, motor_r)
            elif (self.command_pwm >= 0): # 左回り
                motor_l += self.command_pwm/10
                motor_l = motor_l*1.1
                motor_r -= self.command_pwm
                rospy.loginfo("motor_l %lf, motor_r %lf , | Turn Left", motor_l, motor_r)
            GPIO.output(gpio_pin_r, GPIO.HIGH)
            GPIO.output(gpio_pin_l, GPIO.HIGH)
            try:
                p_r.ChangeDutyCycle(motor_r)
                p_l.ChangeDutyCycle(motor_l)
            except:
                rospy.logwarn("DutyCycle is over 100, %lf", self.command_pwm)
            return

        elif self.main == 6:
            motor_r = motor_l = 0
            if (self.cmdvel_from_imu.angular.z <= 0.1 and self.cmdvel_from_imu.angular.z >= -0.1):
                rospy.logwarn("Stop")
                motor_r = motor_l = 0

            elif (self.cmdvel_from_imu.angular.z > 0):
                GPIO.output(gpio_pin_r, GPIO.LOW)
                motor_r += self.cmdvel_from_imu.angular.z
                GPIO.output(gpio_pin_l, GPIO.HIGH)
                motor_l += self.cmdvel_from_imu.angular.z

            elif (self.cmdvel_from_imu.angular.z < 0):
                GPIO.output(gpio_pin_l, GPIO.LOW)
                motor_l += -self.cmdvel_from_imu.angular.z
                GPIO.output(gpio_pin_r, GPIO.HIGH)
                motor_r += -self.cmdvel_from_imu.angular.z

            else:
                rospy.logwarn("Something wrong")

            if(motor_r >= 100): motor_r = 100
            if(motor_r <= -100): motor_r = -100
            if(motor_l >= 100): motor_l = 100
            if(motor_l <= -100): motor_l = -100
            rospy.logwarn("motor_r:%d, motor_l:%d", motor_r, motor_l)
            # GPIO.output(gpio_pin_r, GPIO.HIGH)
            # GPIO.output(gpio_pin_l, GPIO.HIGH)
            p_r.ChangeDutyCycle(motor_r)
            p_l.ChangeDutyCycle(motor_l)
            return

        
    def p_control(self, cur_pos):
        """
        @fn p_control()
        @details P制御
        """
        # p_gain = 0.04
        p_gain = 4.3
        d_gain = 7.0
        current_command = p_gain * (self.ref_pos - cur_pos) + d_gain*((self.ref_pos - cur_pos) - self.prev_command)/self.dt
        self.prev_command = self.ref_pos - cur_pos
        print("cur_pos: ", cur_pos, "diff between cur and ref: ", self.ref_pos - cur_pos)
        return current_command
    
    def cmd_callback(self, msg):
        # 人の位置とサイズを得る
        self.human_info = msg
    
    def imu_callback(self, msg):
        self.cmdvel_from_imu = msg
        
    def joy_callback(self, joy_msg):
        # button[5]で上がる、button[4]で下がる、realsenseの認識距離変更
        newbtn = 0
        max_thresh = 7.0
        min_thresh = 1.0
        self.current_mode.realsense_depth_thresh += float(joy_msg.buttons[5])/5
        self.current_mode.realsense_depth_thresh -= float(joy_msg.buttons[4])/5
        if(max_thresh < self.current_mode.realsense_depth_thresh):
            self.current_mode.realsense_depth_thresh -= 0.2
        elif(min_thresh > self.current_mode.realsense_depth_thresh):
            self.current_mode.realsense_depth_thresh += 0.2

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

        self.current_mode.mode = self.main
        self.mode_pub.publish(self.current_mode)	
        
def main():
    rospy.init_node("tang_control", anonymous=True)
    instance = TangController()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        instance.mode_change()
        rate.sleep()
    rospy.spin()
                
if __name__ == '__main__':
    main()
    GPIO.cleanup()