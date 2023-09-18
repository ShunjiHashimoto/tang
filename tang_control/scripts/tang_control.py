#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import time
import serial
import RPi.GPIO as GPIO
import pigpio
from config import Pin, PID, PWM, HumanFollowParam
# ros
from sensor_msgs.msg import Joy
from tang_msgs.msg import HumanInfo, Modechange, IsDismiss
from tang_teleop.scripts.tang_teleop import TangTeleop
from geometry_msgs.msg import Twist

# modeを選択
GPIO.setmode(GPIO.BCM)
pi = pigpio.pi()

# 回転方向のGPIOピン
GPIO.setup(Pin.r_direction, GPIO.OUT)
GPIO.setup(Pin.l_direction, GPIO.OUT)
GPIO.output(Pin.r_direction, GPIO.HIGH)
GPIO.output(Pin.l_direction, GPIO.HIGH)
# モードのGPIOピン
GPIO.setup(Pin.teleop_mode, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(Pin.follow_mode, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

class TangController():
    def __init__(self):
        # HumanInfo.msg
        self.human_info = HumanInfo()
        self.cmdvel_from_imu = Twist()
        # Modechange.msg
        self.current_mode = Modechange()
        self.current_mode.realsense_depth_thresh = 4.0
        self.main = 0
        self.ref_pos = 0.0
        self.speed = rospy.get_param("/tang_control/speed")
        self.prev_command = 0
        self.command_pwm = 0
        # teleop control
        self.tang_teleop = TangTeleop()

        GPIO.add_event_detect(Pin.teleop_mode, GPIO.FALLING, callback=self.switch_on_callback, bouncetime=250)
        GPIO.add_event_detect(Pin.follow_mode, GPIO.FALLING, callback=self.switch_on_callback, bouncetime=250)

        # subscribe to motor messages on topic "tang_cmd", 追跡対象の位置と大きさ
        self.human_info_sub = rospy.Subscriber('tang_cmd', HumanInfo, self.cmd_callback, queue_size=1)
        self.imu_sub = rospy.Subscriber('cmdvel_from_imu', Twist, self.imu_callback, queue_size=1)
        self.is_dismiss_sub = rospy.Subscriber('is_dismiss', IsDismiss, self.dismiss_callback, queue_size=1)
        self.is_dismiss = IsDismiss()
        # publisher, モードと距離の閾値、赤色検出の閾値をpub
        self.mode_pub = rospy.Publisher('current_param', Modechange, queue_size=1)
    
    def switch_on_callback(self, gpio):
        result = GPIO.input(gpio) # ピンの値を読み取る(HIGH or LOWの1 or 0)
        if(gpio == Pin.teleop_mode and result == 0): 
            self.tang_teleop.main = 0
            print("Manual",gpio)
        if(gpio == Pin.follow_mode and result == 0): 
            self.tang_teleop.main = 1
            print("-------------------------Human",gpio)
        self.current_mode.mode = self.main
        self.mode_pub.publish(self.current_mode)
        return

    def cmd_callback(self, msg):
        # 人の位置とサイズを得る
        self.human_info = msg
        return
    
    def imu_callback(self, msg):
        self.cmdvel_from_imu = msg
        return

    def dismiss_callback(self, msg):
        self.is_dismiss = msg
        return
    
    def nomarilze_speed(self, motor_r, motor_l):
        if(motor_r >= 100): motor_r = 100
        if(motor_r <= -100): motor_r = -100
        if(motor_l >= 100): motor_l = 100
        if(motor_l <= -100): motor_l = -100
        return abs(motor_r), abs(motor_l)

    def p_control(self, cur_pos):
        """
        @fn p_control()
        @details P制御
        """
        current_command = PID.p_gain * (self.ref_pos - cur_pos) + PID.d_gain*((self.ref_pos - cur_pos) - self.prev_command)/PID.dt
        self.prev_command = self.ref_pos - cur_pos
        #print("cur_pos: ", cur_pos, "diff between cur and ref: ", self.ref_pos - cur_pos)
        return current_command

    def send_vel_cmd(self, r_duty, l_duty):
        # PWMを出力
        r_duty, l_duty = self.nomarilze_speed(r_duty, l_duty)
        r_cnv_dutycycle = int((r_duty * 1000000 / 100))
        l_cnv_dutycycle = int((l_duty * 1000000 / 100))
        pi.hardware_PWM(Pin.r_pwm, PWM.freq, r_cnv_dutycycle)
        pi.hardware_PWM(Pin.l_pwm, PWM.freq, l_cnv_dutycycle)
        # rospy.loginfo("r_duty : %d | l_duty: %d", r_duty, l_duty)
        return

    def teleop_control(self):
        motor_l, motor_r = self.tang_teleop.teleop()
        rospy.loginfo(f"motor_l: {motor_l}, motor_r: {motor_r}")
        self.send_vel_cmd(motor_r, motor_l)
        return
        
    def follow_control(self):
        # 速度を距離に従って減衰させる、1m20cm以内で減衰開始する
        if self.is_dismiss.flag: 
            self.send_vel_cmd(0, 0)
            return
        if(self.human_info.human_point.x >= 2.0 or self.human_info.human_point.x < 0.0):
            command_depth = 1.0
        else:
            command_depth = self.human_info.human_point.x / 2.0
        motor_r = self.speed * command_depth
        motor_l = self.speed * command_depth
        # コマンドの制御量を比例制御で決める
        self.command_pwm = self.p_control(self.human_info.human_point.y)
        if (abs(self.command_pwm) > motor_r and self.command_pwm < 0.0):
            self.command_pwm = -self.speed * command_depth
        elif (abs(self.command_pwm) > motor_r and self.command_pwm > 0.0):
            self.command_pwm = self.speed * command_depth
        # 80cm以内であれば止まる
        if self.human_info.human_point.x <= HumanFollowParam.depth_min_thresh and self.human_info.human_point.x > 0.0:
            rospy.logwarn("Stop: %lf", self.human_info.human_point.x)
            motor_r = 0
            motor_l = 0
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
        self.send_vel_cmd(motor_r, motor_l)
        return
    
    def change_control_mode(self):
        if self.tang_teleop.main == 0:
            self.teleop_control()
        elif self.tang_teleop.main == 1:
            self.follow_control()
        return 
            
def main():
    rospy.init_node("tang_control", anonymous=True)
    rate = rospy.Rate(10)
    tang_controller = TangController()
    while not rospy.is_shutdown():
        tang_controller.change_control_mode()
        rate.sleep()
    tang_controller.send_vel_cmd(0, 0)
    rospy.spin()
                
if __name__ == '__main__':
    main()
    GPIO.cleanup(Pin.teleop_mode)
    GPIO.cleanup(Pin.follow_mode)
