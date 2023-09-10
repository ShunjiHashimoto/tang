#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
from datetime import datetime, timedelta
import pigpio
import sys
import csv
import matplotlib.pyplot as plt

pi = pigpio.pi()
length_of_1encoder = 0.12875
csv_file_path = '/home/ubuntu/catkin_ws/src/tang/test/speed_data.csv'

# PIDゲイン
kp = 1.0
ki = 0.005
kd = 0.001
prev_error = 0.0
integral = 0.0
max_output = 1.0

class Motor:
    def __init__(self):
        self.last_gpio_r = None
        self.last_level_r = None
        self.last_gpio_l = None
        self.last_level_l = None
        self.cnt_list = [0, 0]
        self.r_encoder_pin_A = 5
        self.r_encoder_pin_B = 6
        self.r_direction_gpio = 18
        self.r_pwm_pin = 13 
        self.l_encoder_pin_A = 22
        self.l_encoder_pin_B = 27
        self.l_direction_gpio = 17
        self.l_pwm_pin = 12 

    def get_encoder_turn_r(self, gpio, level, tick):
        if gpio == self.last_gpio_r:
            return
        if gpio == self.r_encoder_pin_A and level == 1:
            self.cnt_list[0] += 1 if self.last_level_r == 0 else -1
        self.last_gpio_r = gpio
        self.last_level_r = level

    def get_encoder_turn_l(self, gpio, level, tick):
        if gpio == self.last_gpio_l:
            return
        if gpio == self.l_encoder_pin_A and level == 1:
            self.cnt_list[1] += 1 if self.last_level_l == 0 else -1
        self.last_gpio_l = gpio
        self.last_level_l = level

    def calc_velocity(self, count, time):
        return length_of_1encoder*count/(time*1000)

    def calculate_duty_cicle(self, duty):
        if duty > 90:
            print(f"over duty {duty}")
            return 0
        # 1,000,000で100％
        return int(((duty/100) * 1000000))

    def pid_control(self, error, current_vel, dt):
        global integral, prev_error, ki, kp, kd
        # 累積値
        integral += error
        # 微分値
        derivative = (error - prev_error)/ dt
        prev_error = error
        # 出力計算
        output = kp * error + ki * integral + kd * derivative
        # 出力を限界値の範囲内に収める
        if max_output is not None and output > max_output:
            print(f"max output: {output}")
            output = max_output
        return current_vel + output

    def run(self):
        pi.set_mode(self.r_encoder_pin_A, pigpio.INPUT)
        pi.set_mode(self.r_encoder_pin_B, pigpio.INPUT)
        pi.set_mode(self.r_direction_gpio, pigpio.OUTPUT)
        pi.set_mode(self.l_direction_gpio, pigpio.OUTPUT)
        pi.write(self.r_direction_gpio, pigpio.HIGH)
        pi.write(self.l_direction_gpio, pigpio.HIGH)
        # コールバック関数登録
        pi.set_pull_up_down(self.r_encoder_pin_A, pigpio.PUD_UP)
        pi.set_pull_up_down(self.r_encoder_pin_B, pigpio.PUD_UP)
        pi.callback(self.r_encoder_pin_A, pigpio.EITHER_EDGE, self.get_encoder_turn_r) 
        pi.callback(self.r_encoder_pin_B, pigpio.EITHER_EDGE, self.get_encoder_turn_r) 
        pi.set_pull_up_down(self.l_encoder_pin_A, pigpio.PUD_UP)
        pi.set_pull_up_down(self.l_encoder_pin_B, pigpio.PUD_UP)
        pi.callback(self.l_encoder_pin_A, pigpio.EITHER_EDGE, self.get_encoder_turn_l) 
        pi.callback(self.l_encoder_pin_B, pigpio.EITHER_EDGE, self.get_encoder_turn_l) 

        # PWMパラメータ
        target_vel = float(sys.argv[1])
        initial_pwm = target_vel
        # パラメータ変換
        l_cnv_dutycycle = self.calculate_duty_cicle(initial_pwm)
        r_cnv_dutycycle = self.calculate_duty_cicle(initial_pwm)
        # PWM周波数をHzで指定
        freq = 1000 # [Hz]
        # PWMを出力
        pi.hardware_PWM(self.r_pwm_pin, freq, r_cnv_dutycycle)
        pi.hardware_PWM(self.l_pwm_pin, freq, l_cnv_dutycycle)
        # 処理時間計測
        start_time = datetime.now()
        dt = 0.001 # 0.0001がmax
        # プロット
        plt.figure()
        plt.xlabel("Time [s]")
        plt.ylabel(" ω[rad/s]")
        plt.title("Target ω and Current ω")
        plt.ylim(0, 15)
        time_data = []
        r_vel_data = []
        l_vel_data = []
        target_vel_data = []
        last_time = datetime.now()
        cnt = 0
        prev_cnt_list = [0, 0]
        w_target = 4
        Kp = 1.5
        Kd = 0.001
        Ki = 0.01
        Ke_l = 1.484813
        Ke_r = 1.529544
        radian_1encoder_r = 0.00306
        radian_1encoder_l = 0.003095165176
        error_sum_r = 0
        error_sum_l = 0
        try:
            # 0 r, 1 l
            while (self.cnt_list[1] <= 2030*2):
                if(cnt == 10):
                    # 0.01秒たった
                    end_time = datetime.now()
                    elapsed_time = end_time - start_time
                    elapsed_seconds = round(float(elapsed_time.total_seconds()), 4)
                    last_time = end_time
                    print(elapsed_time)
                    delta_encoder_r = self.cnt_list[0] - prev_cnt_list[0]
                    delta_encoder_l = self.cnt_list[1] - prev_cnt_list[1]
                    w_l = (delta_encoder_l*radian_1encoder_l)/(dt*10)
                    w_r = (delta_encoder_r*radian_1encoder_r)/(dt*10)
                    print("目標はw = 4")
                    print( '\033[31m'+'現在の左と右車輪の速度: '+'\033[0m'+str(w_l)+' : '+str(w_r))
                    prev_cnt_list[0] = self.cnt_list[0]
                    prev_cnt_list[1] = self.cnt_list[1]
                    cnt = 0

                    error_w_l = Kp*(w_target - w_l) + Ki*(error_sum_l) + Kd*(w_target - w_l)/(dt*10)
                    error_w_r =  Kp*(w_target - w_r) + Ki*(error_sum_r) + Kd*(w_target - w_r)/(dt*10)
                    error_sum_l += (w_target - w_l)
                    error_sum_r += (w_target - w_r)
                    print(f"eroor_w_l, r {error_w_l}, {error_w_r}")
                    duty_l = (100*Ke_l*(w_l+error_w_l))/12
                    duty_r = (100*Ke_r*(w_r+error_w_r))/12
                    print(f"duty={duty_l}, {duty_r}")
                    target_vel_data.append(4.0)
                    time_data.append(elapsed_seconds)
                    r_vel_data.append(w_r)
                    l_vel_data.append(w_l)
                    if(duty_l>70 or duty_r>70 or duty_l < 0 or duty_r<0): continue

                    # パラメータ変換
                    l_cnv_dutycycle = self.calculate_duty_cicle(duty_l)
                    r_cnv_dutycycle = self.calculate_duty_cicle(duty_l)
                    # PWMを出力
                    pi.hardware_PWM(self.r_pwm_pin, freq, r_cnv_dutycycle)
                    pi.hardware_PWM(self.l_pwm_pin, freq, l_cnv_dutycycle)

                # print( '\033[31m'+'右車輪の速度: '+'\033[0m'+str(self.cnt_list[0]))
                print( '\033[32m'+'左車輪の速度: '+'\033[0m'+str(self.cnt_list[1]))
                time.sleep(dt)
                cnt += 1
        finally:
            plt.plot(time_data, target_vel_data, label="target")
            plt.plot(time_data, r_vel_data, label="ω_r")
            plt.plot(time_data, l_vel_data, label="ω_l")
            plt.legend()
            plt.grid(True)
            plt.savefig("speed.png")
            pi.hardware_PWM(self.r_pwm_pin, freq, 0)
            pi.hardware_PWM(self.l_pwm_pin, freq, 0)
            pi.stop()

def main():
    motor = Motor()
    motor.run()
    
if __name__ == "__main__":
    main()