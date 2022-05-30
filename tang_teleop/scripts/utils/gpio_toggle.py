#!/usr/bin/env python
# -*- coding: utf-8 -*-

import RPi.GPIO as GPIO
from time import sleep

OUTPUT_GPIO = 27 # LEDに接続するGPIO番号

GPIO.setmode(GPIO.BCM)
GPIO.setup(OUTPUT_GPIO, GPIO.IN)

try:
    print('--- start program ---')
    while True:
        result = GPIO.input(OUTPUT_GPIO)
        print(result)
        # sleep(0.01)
except KeyboardInterrupt:
    pass
finally:
    GPIO.cleanup()
    print('--- stop program ---')
