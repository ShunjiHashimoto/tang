#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import RPi.GPIO as GPIO


def main():
    GPIO.setmode(GPIO.BCM)
    t = 0.1

    # 割り込みイベント設定
    led_D1_pin = 26
    led_D2_pin = 25
    GPIO.setup(led_D1_pin, GPIO.OUT)
    GPIO.setup(led_D2_pin, GPIO.OUT)

    try:
        while (1):
            GPIO.output(led_D1_pin, GPIO.HIGH)
            GPIO.output(led_D2_pin, GPIO.LOW)
            time.sleep(t)
            GPIO.output(led_D2_pin, GPIO.HIGH)
            GPIO.output(led_D1_pin, GPIO.LOW)
            time.sleep(t)
    finally:
        GPIO.cleanup()

if __name__ == '__main__':
    main()
    GPIO.cleanup()
