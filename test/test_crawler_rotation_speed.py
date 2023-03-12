#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import RPi.GPIO as GPIO
import time
import pigpio
import sys

pi = pigpio.pi()
def main():
    # modeを選択
    GPIO.setmode(GPIO.BCM)

    #PWMパラメータ
    l_pwm_pin = 12 
    r_pwm_pin = 13 
    duty = int(sys.argv[1])
    if duty > 90: 
        return 0
    freq = 1000 #PWM周波数をHzで指定

    gpio_pin_r = 18
    gpio_pin_l = 17
    GPIO.setup(gpio_pin_r, GPIO.OUT)
    GPIO.setup(gpio_pin_l, GPIO.OUT)
    GPIO.output(gpio_pin_r, GPIO.HIGH)
    GPIO.output(gpio_pin_l, GPIO.HIGH)

    # パラメータ変換
    l_cnv_dutycycle = int((duty * 1000000 / 100))
    r_cnv_dutycycle = int((duty * 1000000 / 100))

    # PWMを出力
    pi.hardware_PWM(r_pwm_pin, freq, r_cnv_dutycycle)
    pi.hardware_PWM(l_pwm_pin, freq, l_cnv_dutycycle)

    # pi.stop()

main()
GPIO.cleanup()
pi.stop()