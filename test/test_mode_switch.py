#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import RPi.GPIO as GPIO


def main():
    GPIO.setmode(GPIO.BCM)
    t = 0.1

    # 割り込みイベント設定
    auto_mode_pin = 21
    manual_mode_pin = 16
    GPIO.setup(auto_mode_pin, GPIO.IN)
    GPIO.setup(manual_mode_pin, GPIO.IN)

    try:
        while (1):
            auto_pin_value = GPIO.input(auto_mode_pin)
            manual_pin_value = GPIO.input(manual_mode_pin)
            print("automode: {}, manualmode: {}".format(auto_pin_value, manual_pin_value))
            time.sleep(t)
    finally:
        GPIO.cleanup()

if __name__ == '__main__':
    main()
    GPIO.cleanup()
