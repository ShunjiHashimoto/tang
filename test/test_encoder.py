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
        self.r_encoder_pin_A = 27
        self.r_encoder_pin_B = 22
        self.r_direction_gpio = 17
        self.r_pwm_pin = 12
        self.l_encoder_pin_A = 5
        self.l_encoder_pin_B = 6
        self.l_direction_gpio = 18
        self.l_pwm_pin = 13 

    def get_encoder_turn_r(self, gpio, level, tick):
        if gpio == self.last_gpio_r:
            return
        if gpio == self.r_encoder_pin_A and level == 1:
            self.cnt_list[0] += 1 if self.last_level_r == 0 else -1
        self.last_gpio_r = gpio
        self.last_level_r = level

    def get_encoder_turn_l(self, gpio, level, tick):
        # print(f"level: {level}. gpio:{gpio}")
        if gpio == self.last_gpio_l:
            self.last_gpio_l = gpio
            self.last_level_l = level
            return
        if gpio == self.l_encoder_pin_B and level == 1:
            self.cnt_list[1] += 1
        self.last_gpio_l = gpio
        self.last_level_l = level

    def calc_velocity(self, count, time):
        return length_of_1encoder*count/(time*1000)

    def change_vel_to_duty(self, target_vel):
        slope = (0.56 - 0.19) / (50 - 20) - 0.009 # 0.0123333
        return 20 + (target_vel - 0.19) / slope

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
        pi.set_mode(self.l_encoder_pin_A, pigpio.INPUT)
        pi.set_mode(self.l_encoder_pin_B, pigpio.INPUT)
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
        # pi.hardware_PWM(self.r_pwm_pin, freq, r_cnv_dutycycle)
        # pi.hardware_PWM(self.l_pwm_pin, freq, l_cnv_dutycycle)
        # 処理時間計測
        start_time = datetime.now()
        dt = 0.00011
        # プロット
        plt.figure()
        plt.xlabel("Time [s]")
        plt.ylabel("Speed [m/s]")
        plt.title("Target Velocity and Current Velocity")
        plt.ylim(0, 0.5)
        time_data = []
        r_vel_data = []
        l_vel_data = []
        target_vel_data = []
        distance_data = 0.0
        try:
            while (self.cnt_list[1] <= 1400*2):
                end_time = datetime.now()
                elapsed_time = end_time - start_time
                print(elapsed_time)
                elapsed_seconds = round(float(elapsed_time.total_seconds()), 4)
                time_data.append(elapsed_seconds)
                
                r_current_vel = self.calc_velocity(self.cnt_list[0], dt)
                l_current_vel = self.calc_velocity(self.cnt_list[1], dt)
                r_vel_data.append(r_current_vel)
                l_vel_data.append(l_current_vel)
                target_vel_data.append(target_vel)
                # self.cnt_list[0] = 0
                # self.cnt_list[1] = 0
                # PID制御
                # l_pid_output = pid_control(target_vel - l_current_vel, l_current_vel, dt)
                # r_pid_output = pid_control(target_vel - r_current_vel, r_current_vel, dt)
                print( '\033[31m'+'右車輪の速度: '+'\033[0m'+str(self.cnt_list[0]))
                print( '\033[32m'+'左車輪の速度: '+'\033[0m'+str(self.cnt_list[1]))
                # # PWM制御
                # l_pwm = change_vel_to_duty(l_pid_output)
                # r_pwm = change_vel_to_duty(r_pid_output)
                # l_cnv_dutycycle = self.calculate_duty_cicle(20)
                r_cnv_dutycycle = self.calculate_duty_cicle(target_vel)
                pi.hardware_PWM(self.r_pwm_pin, freq, r_cnv_dutycycle)
                pi.hardware_PWM(self.l_pwm_pin, freq, l_cnv_dutycycle)
                distance_data += dt*(l_current_vel + r_current_vel)/2
                
                time.sleep(dt)
        finally:
            # プロット
            plt.plot(time_data, target_vel_data, label="target")
            plt.plot(time_data, r_vel_data, label="r_vel")
            plt.plot(time_data, l_vel_data, label="l_vel")
            plt.legend()
            plt.grid(True)
            plt.savefig("speed.png")
            print(time_data[len(time_data)-1])
            # print(f"進んだ距離:{distance_data*1000}[mm]")
            # print(f"目標距離:{round(target_vel*time_data[len(time_data)-1]*1000, 5)}[mm]")
            # print(f"誤差距離:{round(target_vel*time_data[len(time_data)-1]*1000 - distance_data*1000, 5)}[mm]")
            # print(f"誤差率:{(100*((target_vel*time_data[len(time_data)-1]*1000 - distance_data*1000)/(distance_data*1000)) )}[%]")
            pi.hardware_PWM(self.r_pwm_pin, freq, 0)
            pi.hardware_PWM(self.l_pwm_pin, freq, 0)
            pi.stop()

def main():
    motor = Motor()
    motor.run()
    
if __name__ == "__main__":
    main()