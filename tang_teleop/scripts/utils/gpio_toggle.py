# -*- coding: utf-8 -*-
"""
Created on Sun Aug  2 08:14:04 2020

@author: Souichirou Kikuchi
"""

import RPi.GPIO as GPIO
from time import sleep

LED_GPIO = 4 # LEDに接続するGPIO番号
TACT_GPIO = 17 # Tack Switchに接続するGPIO番号

GPIO.setmode(GPIO.BCM)
GPIO.setup(LED_GPIO, GPIO.OUT)
GPIO.setup(TACT_GPIO, GPIO.IN)

try:
    print('--- start program ---')
    while True:
        if GPIO.input(TACT_GPIO) == GPIO.HIGH:
            GPIO.output(LED_GPIO, GPIO.HIGH)
        else:
            GPIO.output(LED_GPIO, GPIO.LOW)
        sleep(0.01)
except KeyboardInterrupt:
    pass
finally:
    GPIO.cleanup()
    print('--- stop program ---')
