#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import time
import pigpio
from config import Pin, PWM, FOLLOWPID, HumanFollowParam, Control
from motor import Motor
# ros
from tang_msgs.msg import HumanInfo, IsDismiss
from tang_teleop.scripts.tang_teleop import TangTeleop
from geometry_msgs.msg import Twist

class TangController():
    def __init__(self):
        self.pi = pigpio.pi()
        # HumanInfo.msg
        self.human_info = HumanInfo()
        self.cmdvel_from_imu = Twist()
        # 画像の中心座標x, なるべくこの値に近づくように車体を制御する
        self.ref_pos = HumanFollowParam.image_center
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
        # 前回のモード
        self.prev_mode = 0
        # 緊急停止モードの設定
        self.pi.set_mode(Pin.emergency_mode, pigpio.INPUT)
        self.pi.callback(Pin.emergency_mode, pigpio.EITHER_EDGE, self.emergency_button_callback)
        self.debounce_time_micros_emergency = 200000  # 0.2秒 = 200,000マイクロ秒
        self.debounce_time_micros_mode = 2000000
        self.last_tick_emergency= 0
        self.last_tick_mode= 0
        self.prev_target_pos_y = 0.0
        self.demo_prev_time = time.time()
        self.demo_mode_now = 0
    
    def emergency_button_callback(self, gpio, level, tick):
        if pigpio.tickDiff(self.last_tick_emergency, tick) >= self.debounce_time_micros_emergency:
            if(gpio == Pin.emergency_mode and level == 1):
                print(f"緊急停止モード: {gpio}, {level}")
                self.stop_control()
            if(gpio == Pin.emergency_mode and level == 0):
                print(f"緊急停止モード解除: {gpio}, {level}")
                self.tang_teleop.mode = 1
            self.last_tick_emergency = tick
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

    def pid_control(self, cur_pos):
        current_command = FOLLOWPID.p_gain * (self.ref_pos - cur_pos) + FOLLOWPID.d_gain*((self.ref_pos - cur_pos) - self.prev_command)/FOLLOWPID.dt
        self.prev_command = self.ref_pos - cur_pos
        return current_command

    def send_vel_cmd(self, r_duty, l_duty):
        # PWMを出力
        r_duty, l_duty = self.nomarilze_speed(r_duty, l_duty)
        r_cnv_dutycycle = int((r_duty * 1000000 / 100))
        l_cnv_dutycycle = int((l_duty * 1000000 / 100))
        self.motor.pi.hardware_PWM(Pin.pwm_r, PWM.freq, r_cnv_dutycycle)
        self.motor.pi.hardware_PWM(Pin.pwm_l, PWM.freq, l_cnv_dutycycle)
        return
    
    # Joystick操作を使ってモータ制御
    def teleop_control(self):
        motor_l, motor_r = self.tang_teleop.teleop()
        if motor_l >= 0: self.motor.pi.write(Pin.direction_l, pigpio.HIGH)
        if motor_r >= 0: self.motor.pi.write(Pin.direction_r, pigpio.HIGH)
        if motor_l < 0: self.motor.pi.write(Pin.direction_l, pigpio.LOW)
        if motor_r < 0: self.motor.pi.write(Pin.direction_r, pigpio.LOW)
        self.send_vel_cmd(motor_r, motor_l)
        return
    
    # 色検出を使ってモータ制御
    def follow_control(self):
        if self.prev_target_pos_y == 0.0: self.prev_target_pos_y = self.human_info.human_point.y 
        distance_gain = 0.0
        diff = abs(self.human_info.human_point.y - self.prev_target_pos_y)
        self.prev_target_pos_y = self.human_info.human_point.y
        if diff > 400: self.is_dismiss.flag = True
        # 速度を距離に従って減衰させる
        if self.is_dismiss.flag:
            self.motor.run(0.0, 0.0, Control.d_target, Control.alpha_target)
            return
        # 目標速度の計算
        if(self.human_info.max_area <= HumanFollowParam.min_area_thresh):
            distance_gain = 0.0
        else:
            distance_gain = (1 - self.human_info.max_area/HumanFollowParam.max_area_thresh)
            v_target = distance_gain*Control.max_velocity
        # 目標角速度の計算
        w_target = self.pid_control(self.human_info.human_point.y)
        # 対象物が大きすぎる場合は、停止する
        if(distance_gain <= 0.0): 
            v_target = 0.0
            w_target = 0.0
            self.motor.run(v_target, w_target, Control.d_target, Control.alpha_target)
            print(f"color_area: {self.human_info.max_area}, distance_gain: {distance_gain}, w_target: {w_target}, v_target: {v_target}")
            return
        # コマンドの制御量を比例制御で決める
        if(abs(w_target) > Control.max_w):
            if w_target<0: w_target = -Control.max_w
            if w_target>0: w_target = Control.max_w
        print(f"color_area: {self.human_info.max_area}, distance_gain: {distance_gain}, w_target: {w_target}, v_target: {v_target}")
        self.motor.run(v_target, w_target, Control.a_target, Control.alpha_target)
        return
    
    def demo_control(self):
        v_target = 0.0
        if self.demo_mode_now%2 == 0:
            w_target = 1.5
            self.motor.run(v_target, w_target, Control.a_target, Control.alpha_target)
        elif self.demo_mode_now%2 == 1:
            w_target = -1.5
            self.motor.run(v_target, w_target, Control.a_target, Control.alpha_target)
        end_time = time.time() - self.demo_prev_time
        if end_time > 2.0:
            self.demo_mode_now += 1
            self.demo_prev_time = time.time()

    def stop_control(self):
        self.tang_teleop.mode = 99
        self.motor.error_sum = {'v' : 0, 'w' : 0}
        self.motor.prev_error = {'v' : 0, 'w' : 0}
        self.encoder_values = {'r' : 0, 'l' : 0}
        self.prev_encoder_values = {'r' : 0, 'l' : 0}
        return
    
    def start_tang_control(self):
        if self.pi.read(Pin.emergency_mode): 
            print(f"緊急停止モード")
            return
        if self.tang_teleop.demo_mode == 1:
            print(f"デモモード")
            self.demo_control()
            return
        elif self.tang_teleop.mode == 0:
            print(f"遠隔操作")
            self.teleop_control()
        elif self.tang_teleop.mode == 1:
            print(f"追従モード")
            self.follow_control()
        self.prev_mode = self.tang_teleop.mode 
        return 
            
def main():
    rospy.init_node("tang_control", anonymous=True)
    rate = rospy.Rate(100) # 100Hz
    tang_controller = TangController()
    while not rospy.is_shutdown():
        tang_controller.start_tang_control()
        rate.sleep()
    tang_controller.send_vel_cmd(0, 0)
    tang_controller.pi.stop()
    tang_controller.motor.stop()
    rospy.spin()
                
if __name__ == '__main__':
    main()
