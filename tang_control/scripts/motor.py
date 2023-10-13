#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
from datetime import datetime
import pigpio
import sys
import matplotlib.pyplot as plt
from config import Pin, PID, PWM, Fig, Control

class Motor:
    def __init__(self):
        self.pi = pigpio.pi()
        self.last_gpio_r = None
        self.last_level_r = None
        self.last_gpio_l = None
        self.last_level_l = None
        self.encoder_values = {'r' : 0, 'l' : 0}
        self.prev_encoder_values = {'r' : 0, 'l' : 0}
        self.prev_error = {'v' : 0, 'w' : 0}
        self.error_sum = {'v' : 0, 'w' : 0}
        self.prev_time = time.time()
        self.prev_time_for_pwm = time.time()
        self.set_gpio()
        self.start_time = datetime.now()
        # プロット
        self.fig = plt.figure()
        self.ax1 = self.fig.add_subplot(1, 2, 1)
        self.ax2 = self.fig.add_subplot(1, 2, 2)
        
    def set_gpio(self):
        self.pi.set_mode(Pin.encoder_r_A, pigpio.INPUT)
        self.pi.set_mode(Pin.encoder_r_B, pigpio.INPUT)
        self.pi.set_mode(Pin.direction_r, pigpio.OUTPUT)
        self.pi.set_mode(Pin.direction_l, pigpio.OUTPUT)
        self.pi.write(Pin.direction_r, pigpio.HIGH)
        self.pi.write(Pin.direction_l, pigpio.HIGH)
        # コールバック関数登録
        self.pi.set_pull_up_down(Pin.encoder_r_A, pigpio.PUD_UP)
        self.pi.set_pull_up_down(Pin.encoder_r_B, pigpio.PUD_UP)
        self.pi.callback(Pin.encoder_r_A, pigpio.EITHER_EDGE, self.get_encoder_turn_r) 
        self.pi.callback(Pin.encoder_r_B, pigpio.EITHER_EDGE, self.get_encoder_turn_r) 
        self.pi.set_pull_up_down(Pin.encoder_l_A, pigpio.PUD_UP)
        self.pi.set_pull_up_down(Pin.encoder_l_B, pigpio.PUD_UP)
        self.pi.callback(Pin.encoder_l_A, pigpio.EITHER_EDGE, self.get_encoder_turn_l) 
        self.pi.callback(Pin.encoder_l_B, pigpio.EITHER_EDGE, self.get_encoder_turn_l) 

    def get_encoder_turn_r(self, gpio, level, tick):
        if gpio == self.last_gpio_r:
            return
        if gpio == Pin.encoder_r_A and level == 1:
            self.encoder_values['r'] += 1 
        self.last_gpio_r = gpio
        self.last_level_r = level

    def get_encoder_turn_l(self, gpio, level, tick):
        if gpio == self.last_gpio_l:
            return
        if gpio == Pin.encoder_l_A and level == 1:
            self.encoder_values['l'] += 1 
        self.last_gpio_l = gpio
        self.last_level_l = level

    def calculate_duty_cycle(self, duty):
        # 1,000,000で100％
        return int(((duty/100) * 1000000))
    
    def calc_w(self, encoder_val, prev_encoder_val, dt):
        delta_encoder_val = encoder_val - prev_encoder_val
        return (delta_encoder_val*Control.radian_1encoder_r)/dt
    
    def pid_control(self, v_curr, w_curr, v_target, w_target, dt):
        error_v  = v_target - v_curr
        error_w  = w_target - w_curr
        if(error_v < 0): error_v = error_v
        pid_error_v = PID.Kp*error_v + PID.Ki*self.error_sum['v'] + PID.Kd*(error_v - self.prev_error['v'])/dt
        pid_error_w = PID.Kp*error_w + PID.Ki*self.error_sum['w'] + PID.Kd*(error_w - self.prev_error['w'])/dt
        print(f"\033[91merror_v: {error_v:.3f}, 目標速度：{v_target}, 現在速度：{v_curr}, 計算後のerror_v: {pid_error_v:.3f}, error_sum_v: {self.error_sum['v']:.3f}, error_sum_w: {self.error_sum['w']:.3f}\033[0m")
        self.error_sum['v'] += error_v
        self.error_sum['w'] += error_w
        self.prev_error['v'] = error_v
        self.prev_error['w'] = error_w
        if self.error_sum['v'] > PID.max_error_sum_v: self.error_sum['v'] = PID.max_error_sum_v
        if abs(self.error_sum['w']) > PID.max_error_sum_w: 
            if self.error_sum['w'] > 0: self.error_sum['w'] = PID.max_error_sum_w
            if self.error_sum['w'] < 0: self.error_sum['w'] = -PID.max_error_sum_w
        return pid_error_v, pid_error_w
    
    def calc_target_w_i(self, v_target, w_target, a_target, alpha_target, dt):
        # 角速度計算
        measured_w_r = self.calc_w(self.encoder_values['r'], self.prev_encoder_values['r'], dt)
        measured_w_l = self.calc_w(self.encoder_values['l'], self.prev_encoder_values['l'], dt)
        # 車体の速度、角速度計算
        v_est = (measured_w_r*Control.wheel_r + measured_w_l*Control.wheel_r)/2
        w_est = (measured_w_r*Control.wheel_r - measured_w_l*Control.wheel_r)/Control.tread_w
        # エンコーダ値保存
        self.prev_encoder_values['r'] = self.encoder_values['r']
        self.prev_encoder_values['l'] = self.encoder_values['l']
        # PID制御
        pid_error_v, pid_error_w = self.pid_control(v_est, w_est, v_target, w_target,  dt)
        # 各モータの角速度
        w_r = (1/Control.wheel_r)*(v_est + pid_error_v) + (Control.tread_w/(2*Control.wheel_r)*(w_target + pid_error_w))
        w_l = (1/Control.wheel_r)*(v_est + pid_error_v) - (Control.tread_w/(2*Control.wheel_r)*(w_target + pid_error_w))
        # トルク計算
        T_r = (Control.wheel_r/2)*Control.M*a_target + (Control.wheel_r/Control.tread_w)*Control.J*alpha_target
        T_l = (Control.wheel_r/2)*Control.M*a_target - (Control.wheel_r/Control.tread_w)*Control.J*alpha_target
        # 電流計算
        i_r = T_r/Control.Kt_r
        i_l = T_l/Control.Kt_l
        # ログ
        Fig.target_w_data.append(w_target)
        Fig.target_vel_data.append(v_target)
        Fig.vel_data.append(v_est)
        Fig.w_data.append(w_est)
        Fig.target_a_data.append(a_target)
        return w_r, w_l, i_r, i_l
    
    def cal_duty(self, w_r, w_l, i_r, i_l):
        e_r = Control.Ke_r*w_r + Control.R*i_r
        e_l = Control.Ke_l*w_l + Control.R*i_l
        duty_r = 100*e_r/Control.input_v
        duty_l = 100*e_l/Control.input_v
        return duty_r, duty_l
    
    def pwm_control(self, pin, duty):
        cnv_dutycycle = self.calculate_duty_cycle(duty)
        self.pi.hardware_PWM(pin, PWM.freq, cnv_dutycycle)
        return
    
    def motor_control(self, w_r, w_l, i_r, i_l):
        # duty計算
        duty_r, duty_l = self.cal_duty(w_r, w_l, i_r, i_l)
        if duty_r<0: duty_r=0
        if duty_l<0: duty_l=0
        if duty_r > PWM.max_duty: duty_r = PWM.max_duty
        if duty_l > PWM.max_duty: duty_l = PWM.max_duty
        # PWM出力
        self.pwm_control(Pin.pwm_r, duty_r)
        self.pwm_control(Pin.pwm_l, duty_l)
        return
        
    def run(self, v_target, w_target, a_target, alpha_target):
        # 時間更新
        current_time = time.time()
        dt = current_time - self.prev_time
        self.prev_time = time.time()
        # ログ
        elapsed_time = datetime.now() - self.start_time
        elapsed_seconds = round(float(elapsed_time.total_seconds()), 4)
        Fig.time_data.append(elapsed_seconds)
        # モータの目標角速度と電流値を計算
        w_r, w_l, i_r, i_l= self.calc_target_w_i(v_target, w_target, a_target, alpha_target, dt)
        # 0.01秒周期でモータに指令を送る
        if(current_time - dt > 0.01):
            self.motor_control(w_r, w_l, i_r, i_l)
        time.sleep(PID.dt)            
        
    def stop(self):
        self.pwm_control(Pin.pwm_r, 0)
        self.pwm_control(Pin.pwm_l, 0)
        self.ax1.plot(Fig.time_data, Fig.vel_data, color="blue", label="vel_data")
        self.ax1.plot(Fig.time_data, Fig.target_vel_data, color="green", label="target_vel")
        self.ax1.plot(Fig.time_data, Fig.target_a_data, color="black", label="target_a")
        self.ax2.plot(Fig.time_data, Fig.w_data, color="red", label="w_data")
        self.ax2.plot(Fig.time_data, Fig.target_w_data, color="green", label="target_w")
        self.ax1.legend(loc = 'upper right')
        self.ax2.legend(loc = 'upper right')
        self.ax1.grid(True)
        self.ax2.grid(True)
        self.fig.tight_layout()
        now = datetime.now().replace(microsecond=0)
        plt.savefig(f"/home/ubuntu/ros1_ws/src/tang/test/graph/{now}_v={Control.v_target}_a={Control.a_target}_w={Control.w_target}_α={Control.alpha_target}.png")
        self.pi.stop()
            

def main():
    # 目標速度
    v_target = 0.4
    # 目標角速度
    w_target = 0.0001
    # 目標加速度
    a_target = 0.001
    # 目標角加速度
    alpha_target = 0.0001
    motor = Motor()
    motor.run(v_target, w_target, a_target, alpha_target)
    
if __name__ == "__main__":
    main()