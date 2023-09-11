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
        self.prev_error_w = {'r' : 0, 'l' : 0}
        self.error_sum = {'r' : 0, 'l' : 0}
        self.prev_time = time.time()
        self.set_gpio()
        # プロット
        plt.Figure()
        plt.xlabel("Time [s]")
        plt.ylabel(" ω[rad/s]")
        plt.title("Target ω and Current ω")
        plt.ylim(0, 15)
        
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
    
    def pid_control(self, w_r, w_l, dt):
        error_w_r  = Control.w_target - w_r
        error_w_l  = Control.w_target - w_l
        self.error_sum['r'] += error_w_r
        self.error_sum['l'] += error_w_l
        error_w_r = PID.Kp*error_w_r + PID.Ki*self.error_sum['r'] + PID.Kd*(error_w_r - self.prev_error_w['r'])/dt
        error_w_l = PID.Kp*error_w_l + PID.Ki*self.error_sum['l'] + PID.Kd*(error_w_l - self.prev_error_w['l'])/dt
        return error_w_r, error_w_l
    
    def cal_duty(self, Ke, w, error_w):
        return (100*Ke*(w + error_w))/Control.input_v
    
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
                    w_r = self.calc_w(self.encoder_values['r'], self.prev_encoder_values['r'], dt)
                    w_l = self.calc_w(self.encoder_values['l'], self.prev_encoder_values['l'], dt)
                    self.prev_encoder_values['r'] = self.encoder_values['r']
                    self.prev_encoder_values['l'] = self.encoder_values['l']
                    error_w_r, error_w_l = self.pid_control(w_r, w_l, dt)
                    # duty計算
                    duty_r = self.cal_duty(Control.Ke_r, w_r, error_w_r)
                    duty_l = self.cal_duty(Control.Ke_l, w_l, error_w_l)
                    self.prev_error_w['r'] = error_w_r
                    self.prev_error_w['l'] = error_w_l
                    # PWM出力
                    self.pwm_control(Pin.pwm_r, duty_r)
                    self.pwm_control(Pin.pwm_l, duty_l)
                    # ログ
                    Fig.target_vel_data.append(Control.w_target)
                    Fig.time_data.append(elapsed_seconds)
                    Fig.vel_data_r.append(w_r)
                    Fig.vel_data_l.append(w_l)
                    print(f"目標角速度 w = {Control.w_target}")
                    print( '\033[31m'+'現在の左と右車輪の速度: '+'\033[0m'+str(w_l)+' : '+str(w_r))
                    print(f"error w_l, r {error_w_l}, {error_w_r}")
                    print(f"duty={duty_l}, {duty_r}")

                # print( '\033[31m'+'右車輪の速度: '+'\033[0m'+str(self.encoder_values['r']))
                print( '\033[32m'+'左車輪の速度: '+'\033[0m'+str(self.encoder_values['l']))
                time.sleep(PID.dt)
        finally:
            plt.plot(Fig.time_data, Fig.target_vel_data, label="target")
            plt.plot(Fig.time_data, Fig.vel_data_r, label="ω_r")
            plt.plot(Fig.time_data, Fig.vel_data_l, label="ω_l")
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