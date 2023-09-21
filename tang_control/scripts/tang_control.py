#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import time
import pigpio
from config import Pin, PWM, FOLLOWPID
from motor import Motor
# ros
from tang_msgs.msg import HumanInfo, IsDismiss
from tang_teleop.scripts.tang_teleop import TangTeleop
from geometry_msgs.msg import Twist

# modeを選択
GPIO.setmode(GPIO.BCM)
# pi = pigpio.pi()
# モードのGPIOピン
GPIO.setup(Pin.teleop_mode, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(Pin.follow_mode, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

class TangController():
    def __init__(self):
        self.pi = pigpio.pi()
        # モードのGPIOピン
        self.pi.set_mode(Pin.teleop_mode, pigpio.INPUT)
        self.pi.set_mode(Pin.follow_mode, pigpio.INPUT)
        self.pi.callback(Pin.teleop_mode, pigpio.FALLING, self.switch_on_callback, bouncetime=250)
        self.pi.callback(Pin.teleop_mode, pigpio.FALLING, self.switch_on_callback, bouncetime=250)
        # HumanInfo.msg
        self.human_info = HumanInfo()
        self.cmdvel_from_imu = Twist()
        self.main = 0
        self.ref_pos = 150
        self.speed = rospy.get_param("/tang_control/speed")
        self.prev_command = 0
        # teleop control
        self.tang_teleop = TangTeleop()
        # motor
        self.motor = Motor()
        # subscribe to motor messages on topic "tang_cmd", 追跡対象の位置と大きさ
        self.human_info_sub = rospy.Subscriber('tang_cmd', HumanInfo, self.cmd_callback, queue_size=1)
        self.imu_sub = rospy.Subscriber('cmdvel_from_imu', Twist, self.imu_callback, queue_size=1)
        self.is_dismiss_sub = rospy.Subscriber('is_dismiss', IsDismiss, self.dismiss_callback, queue_size=1)
        self.is_dismiss = IsDismiss()
    
    def switch_on_callback(self, gpio):
        result = self.pi.read(gpio) # ピンの値を読み取る(HIGH or LOWの1 or 0)
        if(gpio == Pin.teleop_mode and result == 0): 
            self.tang_teleop.main = 0
            print("Manual",gpio)
        if(gpio == Pin.follow_mode and result == 0): 
            self.tang_teleop.main = 1
            print("--Human",gpio)
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
        current_command = FOLLOWPID.p_gain * (self.ref_pos - cur_pos) + FOLLOWPID.d_gain*((self.ref_pos - cur_pos) - self.prev_command)/FOLLOWPID.dt
        self.prev_command = self.ref_pos - cur_pos
        #print("cur_pos: ", cur_pos, "diff between cur and ref: ", self.ref_pos - cur_pos)
        return current_command

    def send_vel_cmd(self, r_duty, l_duty):
        # PWMを出力
        r_duty, l_duty = self.nomarilze_speed(r_duty, l_duty)
        r_cnv_dutycycle = int((r_duty * 1000000 / 100))
        l_cnv_dutycycle = int((l_duty * 1000000 / 100))
        self.motor.pi.hardware_PWM(Pin.pwm_r, PWM.freq, r_cnv_dutycycle)
        self.motor.pi.hardware_PWM(Pin.pwm_l, PWM.freq, l_cnv_dutycycle)
        # rospy.loginfo("r_duty : %d | l_duty: %d", r_duty, l_duty)
        return
    
    # Joystick操作を使ってモータ制御
    def teleop_control(self):
        motor_l, motor_r = self.tang_teleop.teleop()
        if motor_l >= 0: self.motor.pi.write(Pin.direction_l, pigpio.HIGH)
        if motor_r >= 0: self.motor.pi.write(Pin.direction_r, pigpio.HIGH)
        if motor_l < 0: self.motor.pi.write(Pin.direction_l, pigpio.LOW)
        if motor_r < 0: self.motor.pi.write(Pin.direction_r, pigpio.LOW)
        rospy.loginfo(f"motor_l: {motor_l}, motor_r: {motor_r}")
        self.send_vel_cmd(motor_r, motor_l)
        return
    
    # 色検出を使ってモータ制御
    def follow_control(self):
        # 速度を距離に従って減衰させる、1m20cm以内で減衰開始する
        if self.is_dismiss.flag: 
            self.send_vel_cmd(0, 0)
            return
        if(self.human_info.human_point.x >= 2.0 or self.human_info.human_point.x < 0.0):
            command_depth = 1.0
        else:
            command_depth = self.human_info.human_point.x / 2.0
        # コマンドの制御量を比例制御で決める
        print(f"self.human_info.human_point.y: {self.human_info.human_point.y}, {self.human_info.human_point.x}")
        # TODO: diffに応じて車体の速度と角速度を決める
        diff_x = self.p_control(self.human_info.human_point.x)
        w_target = diff_x*0.001
        v_target = 0.4
        print(f"diff_x: {diff_x}")
        self.motor.run(v_target, w_target, a_target = 0.001, alpha_target = 0.0001)
        return
    
    def start_tang_control(self):
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
        tang_controller.start_tang_control()
        rate.sleep()
    tang_controller.send_vel_cmd(0, 0)
    tang_controller.pi.stop()
    rospy.spin()
                
if __name__ == '__main__':
    main()
