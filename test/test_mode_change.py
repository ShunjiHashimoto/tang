# !/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import Jetson.GPIO as GPIO


def main():
    GPIO.setmode(GPIO.BOARD)
    t = 0.1

    # 割り込みイベント設定
    emergency_mode_pin = 37
    GPIO.setup(emergency_mode_pin, GPIO.IN)
    emergency_output_pin = 35
    GPIO.setup(emergency_output_pin, GPIO.OUT, initial=GPIO.HIGH)

    try:
        while (1):
            pin_value = GPIO.input(emergency_mode_pin)
            print("pin_value: {}".format(pin_value))
            time.sleep(t)
    finally:
        GPIO.cleanup()

if __name__ == '__main__':
    main()
    GPIO.cleanup()