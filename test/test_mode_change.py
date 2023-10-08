#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import pigpio

emergency_last_level = 0
emergency_switch_pin = 26
debounce_time_micros = 200000  # 0.2秒 = 200,000マイクロ秒
last_tick = 0

def emergency_button_callback(gpio, level, tick):
    global last_tick
    if pigpio.tickDiff(last_tick, tick) >= debounce_time_micros:
        print(f"gpio: {gpio}, level{level}")
        if(gpio == emergency_switch_pin and level == 0):
            print(f"Emergency: {gpio}, {level}")
        if(gpio == emergency_switch_pin and level == 1):
            print(f"Emergency解除: {gpio}, {level}")
        last_tick = tick
    return

def main():
    gpio = pigpio.pi()
    t = 0.1

    # 割り込みイベント設定
    auto_mode_pin = 21
    manual_mode_pin = 16

    gpio.set_mode(auto_mode_pin, pigpio.INPUT)
    gpio.set_mode(manual_mode_pin, pigpio.INPUT)
    gpio.set_mode(emergency_switch_pin, pigpio.INPUT)
    gpio.callback(emergency_switch_pin, pigpio.EITHER_EDGE, emergency_button_callback)

    try:
        while (1):
            auto_pin_value = gpio.read(auto_mode_pin)
            manual_pin_value = gpio.read(manual_mode_pin)
            emergency_pin_value = gpio.read(emergency_switch_pin)
            print("automode: {}, manualmode: {}, emergency: {}".format(auto_pin_value, manual_pin_value, emergency_pin_value))
            time.sleep(t)
    finally:
        GPIO.cleanup()

if __name__ == '__main__':
    main()
    GPIO.cleanup()
