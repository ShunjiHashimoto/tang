#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
from datetime import datetime
import pigpio
import sys
import matplotlib.pyplot as plt
from test_config import Pin, PID, PWM, FIG, Control

pi = pigpio.pi()

class Motor:
    def __init__(self):
        self.last_gpio_r = None
        self.last_level_r = None
        self.last_gpio_l = None
        self.last_level_l = None
        self.encoder_values = {'r' : 0, 'l' : 0}
        self.set_gpio()
        # プロット
        plt.figure()
        plt.xlabel("Time [s]")
        plt.ylabel(" ω[rad/s]")
        plt.title("Target ω and Current ω")
        plt.ylim(0, 15)
        
    def set_gpio(self):
        pi.set_mode(Pin.encoder_r_A, pigpio.INPUT)
        pi.set_mode(Pin.encoder_r_B, pigpio.INPUT)
        pi.set_mode(Pin.direction_r, pigpio.OUTPUT)
        pi.set_mode(Pin.direction_l, pigpio.OUTPUT)
        pi.write(Pin.direction_r, pigpio.HIGH)
        pi.write(Pin.direction_l, pigpio.HIGH)
        # コールバック関数登録
        pi.set_pull_up_down(Pin.encoder_r_A, pigpio.PUD_UP)
        pi.set_pull_up_down(Pin.encoder_r_B, pigpio.PUD_UP)
        pi.callback(Pin.encoder_r_A, pigpio.EITHER_EDGE, self.get_encoder_turn_r) 
        pi.callback(Pin.encoder_r_B, pigpio.EITHER_EDGE, self.get_encoder_turn_r) 
        pi.set_pull_up_down(Pin.encoder_l_A, pigpio.PUD_UP)
        pi.set_pull_up_down(Pin.encoder_l_B, pigpio.PUD_UP)
        pi.callback(Pin.encoder_l_B, pigpio.EITHER_EDGE, self.get_encoder_turn_l) 
        pi.callback(Pin.encoder_l_A, pigpio.EITHER_EDGE, self.get_encoder_turn_l) 

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

    def calculate_duty_cicle(self, duty):
        if duty > 90:
            print(f"over duty {duty}")
            return 0
        # 1,000,000で100％
        return int(((duty/100) * 1000000))
    
    def run(self):
        # PWMパラメータ
        target_vel = float(sys.argv[1])
        initial_pwm = target_vel
        # パラメータ変換
        l_cnv_dutycycle = self.calculate_duty_cicle(initial_pwm)
        r_cnv_dutycycle = self.calculate_duty_cicle(initial_pwm)
        # PWM周波数をHzで指定
        freq = 1000 # [Hz]
        # PWMを出力
        pi.hardware_PWM(Pin.pwm_r, freq, r_cnv_dutycycle)
        pi.hardware_PWM(Pin.pwm_l, freq, l_cnv_dutycycle)
        # 処理時間計測
        start_time = datetime.now()
        last_time = datetime.now()
        cnt = 0
        prev_cnt_list = [0, 0]
        error_sum_r = 0
        error_sum_l = 0
        try:
            # 0 r, 1 l
            while (self.encoder_values['l'] <= 2030*2):
                if(cnt == 10):
                    # 0.01秒たった
                    end_time = datetime.now()
                    elapsed_time = end_time - start_time
                    elapsed_seconds = round(float(elapsed_time.total_seconds()), 4)
                    last_time = end_time
                    print(elapsed_time)
                    delta_encoder_r = self.encoder_values['r'] - prev_cnt_list[0]
                    delta_encoder_l = self.encoder_values['l'] - prev_cnt_list[1]
                    w_l = (delta_encoder_l*Control.radian_1encoder_l)/(PID.dt*10)
                    w_r = (delta_encoder_r*Control.radian_1encoder_r)/(PID.dt*10)
                    print("目標はw = 4")
                    print( '\033[31m'+'現在の左と右車輪の速度: '+'\033[0m'+str(w_l)+' : '+str(w_r))
                    prev_cnt_list[0] = self.encoder_values['r']
                    prev_cnt_list[1] = self.encoder_values['l']
                    cnt = 0

                    error_w_l = PID.Kp*(Control.w_target - w_l) + PID.Ki*(error_sum_l) + PID.Kd*(Control.w_target - w_l)/(PID.dt*10)
                    error_w_r =  PID.Kp*(Control.w_target - w_r) + PID.Ki*(error_sum_r) + PID.Kd*(Control.w_target - w_r)/(PID.dt*10)
                    error_sum_l += (Control.w_target - w_l)
                    error_sum_r += (Control.w_target - w_r)
                    print(f"eroor_w_l, r {error_w_l}, {error_w_r}")
                    duty_l = (100*Control.Ke_l*(w_l+error_w_l))/12
                    duty_r = (100*Control.Ke_r*(w_r+error_w_r))/12
                    print(f"duty={duty_l}, {duty_r}")
                    FIG.target_vel_data.append(4.0)
                    FIG.time_data.append(elapsed_seconds)
                    FIG.vel_data_r.append(w_r)
                    FIG.vel_data_l.append(w_l)
                    if(duty_l>70 or duty_r>70 or duty_l < 0 or duty_r<0): continue

                    # パラメータ変換
                    l_cnv_dutycycle = self.calculate_duty_cicle(duty_l)
                    r_cnv_dutycycle = self.calculate_duty_cicle(duty_l)
                    # PWMを出力
                    pi.hardware_PWM(Pin.pwm_r, freq, r_cnv_dutycycle)
                    pi.hardware_PWM(Pin.pwm_l, freq, l_cnv_dutycycle)

                # print( '\033[31m'+'右車輪の速度: '+'\033[0m'+str(self.encoder_values['r']))
                print( '\033[32m'+'左車輪の速度: '+'\033[0m'+str(self.encoder_values['l']))
                time.sleep(PID.dt)
                cnt += 1
        finally:
            plt.plot(FIG.time_data, FIG.target_vel_data, label="target")
            plt.plot(FIG.time_data, FIG.vel_data_r, label="ω_r")
            plt.plot(FIG.time_data, FIG.vel_data_l, label="ω_l")
            plt.legend()
            plt.grid(True)
            plt.savefig("speed.png")
            pi.hardware_PWM(Pin.pwm_r, freq, 0)
            pi.hardware_PWM(Pin.pwm_l, freq, 0)
            pi.stop()

def main():
    motor = Motor()
    motor.run()
    
if __name__ == "__main__":
    main()