#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
from datetime import datetime
import pigpio
import sys
import matplotlib.pyplot as plt
from test_config import Pin, PID, PWM, Fig, Control

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
        self.set_gpio()
        # プロット
        plt.xlabel("Time [s]")
        plt.ylabel("v[m/s]")
        plt.title("Target v and Current v")
        plt.ylim(0, 1)
        
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
        self.pi.callback(Pin.encoder_l_B, pigpio.EITHER_EDGE, self.get_encoder_turn_l) 
        self.pi.callback(Pin.encoder_l_A, pigpio.EITHER_EDGE, self.get_encoder_turn_l) 

    def get_encoder_turn_r(self, gpio, level, tick):
        if gpio == self.last_gpio_r:
            return
        if gpio == Pin.encoder_r_A and level == 1:
            self.encoder_values['r'] += 1 if self.last_level_r == 0 else -1
        self.last_gpio_r = gpio
        self.last_level_r = level

    def get_encoder_turn_l(self, gpio, level, tick):
        if gpio == self.last_gpio_l:
            return
        if gpio == Pin.encoder_l_A and level == 1:
            self.encoder_values['l'] += 1 if self.last_level_l == 0 else -1
        self.last_gpio_l = gpio
        self.last_level_l = level

    def calculate_duty_cycle(self, duty):
        if duty > 70:
            print(f"over duty {duty}")
            return 0
        # 1,000,000で100％
        return int(((duty/100) * 1000000))
    
    def calc_w(self, encoder_val, prev_encoder_val, dt):
        delta_encoder_val = encoder_val - prev_encoder_val
        return (delta_encoder_val*Control.radian_1encoder_r)/dt
    
    def pid_control(self, v_curr, w_curr, dt):
        error_v  = Control.v_target - v_curr
        error_w  = Control.w_target - w_curr
        print(f"error_v: {error_v} w_curr: {w_curr}")
        self.error_sum['v'] += error_v
        self.error_sum['w'] += error_w
        self.prev_error['v'] = error_v
        self.prev_error['w'] = error_w
        pid_error_v = PID.Kp*error_v + PID.Ki*self.error_sum['v'] + PID.Kd*(error_v - self.prev_error['v'])/dt
        pid_error_w = PID.Kp*error_w + PID.Ki*self.error_sum['w'] + PID.Kd*(error_w - self.prev_error['w'])/dt
        print(f"pid_error_v: {pid_error_v}, pid_error_w: {pid_error_w}")
        return pid_error_v, pid_error_w
    
    def cal_duty(self, w_r, w_l, i_r, i_l):
        print(f"Control.R*i_r: { Control.R*i_r},  Control.R*i_l: { Control.R*i_l}")
        print(f"Control.Ke_r*w_r: {Control.Ke_r*w_r}, Control.Ke_l*w_l: {Control.Ke_l*w_l}")
        e_r = Control.Ke_r*w_r + Control.R*i_r
        e_l = Control.Ke_l*w_l + Control.R*i_l
        print(f"e_r: {e_r}, e_l: {e_l}")
        duty_r = 100*e_r/Control.input_v
        duty_l = 100*e_l/Control.input_v
        return duty_r, duty_l
    
    def pwm_control(self, pin, duty):
        cnv_dutycycle = self.calculate_duty_cycle(duty)
        self.pi.hardware_PWM(pin, PWM.freq, cnv_dutycycle)
        return
        
    def run(self):
        # PWMパラメータ
        target_vel = float(sys.argv[1])
        initial_pwm = target_vel
        # PWMを出力
        self.pwm_control(Pin.pwm_r, initial_pwm)
        self.pwm_control(Pin.pwm_l, initial_pwm)
        # 処理時間計測
        start_time = datetime.now()
        self.prev_encoder_values['r'] = self.encoder_values['r']
        self.prev_encoder_values['l'] = self.encoder_values['l']
        try:
            while (self.encoder_values['l'] <= Control.encoder_1rotation_l*Control.rotation_num):
                current_time = time.time()
                dt = current_time - self.prev_time
                if(dt >= 0.01):
                    # 処理時間計測
                    self.prev_time = current_time
                    end_time = datetime.now()
                    elapsed_time = end_time - start_time
                    elapsed_seconds = round(float(elapsed_time.total_seconds()), 4)
                    # 角速度計算
                    measured_w_r = self.calc_w(self.encoder_values['r'], self.prev_encoder_values['r'], dt)
                    measured_w_l = self.calc_w(self.encoder_values['l'], self.prev_encoder_values['l'], dt)
                    print(f"measured_w_r: {measured_w_r}, measured_w_l: {measured_w_l}")
                    # 車体の速度、角速度計算
                    v_est = (measured_w_r*Control.wheel_r + measured_w_l*Control.wheel_r)/2
                    w_est = (measured_w_r*Control.wheel_r - measured_w_l*Control.wheel_r)/Control.tread_w
                    # エンコーダ値保存
                    self.prev_encoder_values['r'] = self.encoder_values['r']
                    self.prev_encoder_values['l'] = self.encoder_values['l']
                    # print(f"prev_encoder_values[r][l]: {self.prev_encoder_values['r']}, {self.prev_encoder_values['l']}")
                    # PID制御
                    pid_error_v, pid_error_w = self.pid_control(v_est, w_est, dt)
                    # 各モータの角速度
                    w_r = (1/Control.wheel_r)*(Control.v_target + pid_error_v) + (Control.tread_w/(2*Control.wheel_r)*(Control.w_target + pid_error_w))
                    w_l = (1/Control.wheel_r)*(Control.v_target + pid_error_v) - (Control.tread_w/(2*Control.wheel_r)*(Control.w_target + pid_error_w))
                    print(f"w_r: {w_r}, w_l: {w_l}")
                    # トルク計算
                    T_r = (Control.wheel_r/2)*Control.M*Control.a_target + (Control.wheel_r/Control.tread_w)*Control.J*Control.alpha_target
                    T_l = (Control.wheel_r/2)*Control.M*Control.a_target - (Control.wheel_r/Control.tread_w)*Control.J*Control.alpha_target
                    # 電流計算
                    i_r = T_r/Control.Kt_r
                    i_l = T_l/Control.Kt_l
                    # print(f"i_r: {i_r}, i_l: {i_l}")
                    # duty計算
                    duty_r, duty_l = self.cal_duty(w_r, w_l, i_r, i_l)
                    if(duty_r > PWM.max_duty or duty_l > PWM.max_duty or duty_r < 0 or duty_l < 0):
                        print(f"over duty: r={duty_r}, l={duty_l}")
                        continue
                    # PWM出力
                    self.pwm_control(Pin.pwm_r, duty_r)
                    self.pwm_control(Pin.pwm_l, duty_l)
                    # ログ
                    Fig.time_data.append(elapsed_seconds)
                    Fig.target_vel_data.append(Control.v_target)
                    Fig.vel_data.append(v_est)
                    # Fig.w_data.append(w_est)
                    # print(f"目標角速度 v = {Control.v_target}")
                    # print( '\033[31m'+'推定した車体速度: '+'\033[0m'+ str(v_est))
                    # print(f"duty_r={duty_r}, duty_l={duty_l}")
                # print( '\033[31m'+'右車輪の速度: '+'\033[0m'+str(self.encoder_values['r']))
                # print( '\033[32m'+'左車輪の速度: '+'\033[0m'+str(self.encoder_values['l']))
                time.sleep(PID.dt)
            # self.error_sum['v'] = 0
            # self.error_sum['w'] = 0
            self.prev_error['v'] = 0
            self.prev_error['w'] = 0
            Control.a_target  = -0.5
            Control.v_target  = 0.2
            self.prev_encoder_values['r'] = self.encoder_values['r']
            self.prev_encoder_values['l'] = self.encoder_values['l']
            while (self.encoder_values['l'] <= Control.encoder_1rotation_l*8):
                current_time = time.time()
                dt = current_time - self.prev_time
                if(dt >= 0.01):
                    print(f"dt: {dt}")
                    # 処理時間計測
                    self.prev_time = current_time
                    end_time = datetime.now()
                    elapsed_time = end_time - start_time
                    elapsed_seconds = round(float(elapsed_time.total_seconds()), 4)
                    # 角速度計算
                    measured_w_r = self.calc_w(self.encoder_values['r'], self.prev_encoder_values['r'], dt)
                    measured_w_l = self.calc_w(self.encoder_values['l'], self.prev_encoder_values['l'], dt)
                    print(f"measured_w_r: {measured_w_r}, measured_w_l: {measured_w_l}")
                    # 車体の速度、角速度計算
                    v_est = (measured_w_r*Control.wheel_r + measured_w_l*Control.wheel_r)/2
                    w_est = (measured_w_r*Control.wheel_r - measured_w_l*Control.wheel_r)/Control.tread_w
                    # エンコーダ値保存
                    self.prev_encoder_values['r'] = self.encoder_values['r']
                    self.prev_encoder_values['l'] = self.encoder_values['l']
                    print(f"prev_encoder_values[r][l]: {self.prev_encoder_values['r']}, {self.prev_encoder_values['l']}")
                    # PID制御
                    pid_error_v, pid_error_w = self.pid_control(v_est, w_est, dt)
                    # 各モータの角速度
                    w_r = (1/Control.wheel_r)*(v_est + pid_error_v) + (Control.tread_w/(2*Control.wheel_r)*(Control.w_target + pid_error_w))
                    w_l = (1/Control.wheel_r)*(v_est + pid_error_v) - (Control.tread_w/(2*Control.wheel_r)*(Control.w_target + pid_error_w))
                    print(f"w_r: {w_r}, w_l: {w_l}")
                    # トルク計算
                    T_r = (Control.wheel_r/2)*Control.M*Control.a_target + (Control.wheel_r/Control.tread_w)*Control.J*Control.alpha_target
                    T_l = (Control.wheel_r/2)*Control.M*Control.a_target - (Control.wheel_r/Control.tread_w)*Control.J*Control.alpha_target
                    # 電流計算
                    i_r = abs(T_r/Control.Kt_r)
                    i_l = abs(T_l/Control.Kt_l)
                    print(f"i_r: {i_r}, i_l: {i_l}")
                    # duty計算
                    duty_r, duty_l = self.cal_duty(w_r, w_l, i_r, i_l)
                    if(duty_r > PWM.max_duty or duty_l > PWM.max_duty or duty_r < 0 or duty_l < 0):
                        print(f"over duty: r={duty_r}, l={duty_l}")
                        continue
                    # PWM出力
                    self.pwm_control(Pin.pwm_r, duty_r)
                    self.pwm_control(Pin.pwm_l, duty_l)
                    # ログ
                    Fig.time_data.append(elapsed_seconds)
                    Fig.target_vel_data.append(Control.v_target)
                    Fig.vel_data.append(v_est)
                    # Fig.w_data.append(w_est)
                    # print(f"目標角速度 v = {Control.v_target}")
                    print( '\033[31m'+'推定した車体速度: '+'\033[0m'+ str(v_est))
                    print(f"duty_r={duty_r}, duty_l={duty_l}")
                # print( '\033[31m'+'右車輪の速度: '+'\033[0m'+str(self.encoder_values['r']))
                # print( '\033[32m'+'左車輪の速度: '+'\033[0m'+str(self.encoder_values['l']))
                time.sleep(PID.dt)
        finally:
            plt.plot(Fig.time_data, Fig.target_vel_data, label="target")
            plt.plot(Fig.time_data, Fig.vel_data, label="v")
            # plt.plot(Fig.time_data, Fig.w_data, label="ω")
            plt.legend()
            plt.grid(True)
            plt.savefig("speed.png")
            self.pwm_control(Pin.pwm_r, 0)
            self.pwm_control(Pin.pwm_l, 0)
            self.pi.stop()

def main():
    motor = Motor()
    motor.run()
    
if __name__ == "__main__":
    main()