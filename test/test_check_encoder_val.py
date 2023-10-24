#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import pigpio
import sys

pi = pigpio.pi()
length_of_1encoder = 0.12875

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
        self.l_encoder_pin_A = 6
        self.l_encoder_pin_B = 5
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
        if gpio == self.last_gpio_l:
            return
        if gpio == self.l_encoder_pin_A and level == 1:
            self.cnt_list[1] += 1 if self.last_level_l == 0 else -1
        self.last_gpio_l = gpio
        self.last_level_l = level

    def calculate_duty_cicle(self, duty):
        if duty > 90:
            print(f"over duty {duty}")
            return 0
        # 1,000,000で100％
        return int(((duty/100) * 1000000))

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
        # PWM周波数をHzで指定
        freq = 1000 # [Hz]
        dt = 0.00011
        try:
            while (self.cnt_list[0] <= 2030*2):
                print( '\033[31m'+'右車輪のエンコーダ値: '+'\033[0m'+str(self.cnt_list[0]))
                print( '\033[32m'+'左車輪のエンコーダ値: '+'\033[0m'+str(self.cnt_list[1]))
                l_cnv_dutycycle = self.calculate_duty_cicle(target_vel)
                r_cnv_dutycycle = self.calculate_duty_cicle(target_vel)
                pi.hardware_PWM(self.r_pwm_pin, freq, r_cnv_dutycycle)
                pi.hardware_PWM(self.l_pwm_pin, freq, l_cnv_dutycycle)
                time.sleep(dt)
        finally:
            pi.hardware_PWM(self.r_pwm_pin, freq, 0)
            pi.hardware_PWM(self.l_pwm_pin, freq, 0)
            pi.stop()

def main():
    motor = Motor()
    motor.run()
    
if __name__ == "__main__":
    main()