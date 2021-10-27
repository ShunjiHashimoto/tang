#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import time
import RPi.GPIO as GPIO


def main():
    # modeを選択
    GPIO.setmode(GPIO.BCM)
    gpio_pin_l = 17
    gpio_pin_r = 18
    GPIO.setup(gpio_pin_r, GPIO.OUT)
    GPIO.setup(gpio_pin_l, GPIO.OUT)

    # アナログピン, right)ANA2:33, left)ANA1:32
    pwm_pin_r = 13
    GPIO.setup(pwm_pin_r, GPIO.OUT)
    p_r = GPIO.PWM(pwm_pin_r, 50)
    p_r.start(0)

    while (1):
        time.sleep(1)
        # デジタル出力ピンを設定, 回転方向を決められる
        # DIG1 = 11(LEFT), DIG2 = 12(RIGHT)
        print("gpio high")
        GPIO.output(gpio_pin_r, GPIO.HIGH)
        GPIO.output(gpio_pin_l, GPIO.HIGH)
        p_r.ChangeDutyCycle(40)
        time.sleep(6)

        p_r.stop()
        print("stop")
        time.sleep(1)

        p_r.start(0)
        GPIO.output(gpio_pin_r, GPIO.LOW)
        GPIO.output(gpio_pin_l, GPIO.LOW)
        print("gpio low")
        p_r.ChangeDutyCycle(40)
        time.sleep(6)


if __name__ == '__main__':
    main()
    GPIO.cleanup()
