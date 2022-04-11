#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import sys
import time
import RPi.GPIO as GPIO
# import I2C_LCD_driver

def main():
    # modeを選択
    GPIO.setmode(GPIO.BCM)
    gpio_pin_l = 18
    gpio_pin_r = 17
    GPIO.setup(gpio_pin_r, GPIO.OUT)
    GPIO.setup(gpio_pin_l, GPIO.OUT)
    GPIO.setup(11, GPIO.OUT, initial = GPIO.LOW)
    GPIO.setup(19, GPIO.OUT, initial = GPIO.LOW)
    GPIO.setup(21, GPIO.OUT, initial = GPIO.LOW)
    GPIO.setup(16, GPIO.OUT, initial = GPIO.LOW)

    # アナログピン, ANA2(RIGHT) = 32
    pwm_pin_r = 12
    GPIO.setup(pwm_pin_r, GPIO.OUT)
    p_r = GPIO.PWM(pwm_pin_r, 40)
    p_r.start(0)

    pwm_pin_l = 13
    GPIO.setup(pwm_pin_l, GPIO.OUT)
    p_l = GPIO.PWM(pwm_pin_l, 40)
    p_l.start(0)
    # lcd = I2C_LCD_driver.lcd()
    try:
        while (1):
            time.sleep(1)
            # デジタル出力ピンを設定, 回転方向を決められる
            # DIG1 = 27(LEFT), DIG2 = 18(RIGHT)
            print("gpio high")
	    # lcd.lcd_display_string("High")
            GPIO.output(gpio_pin_r, GPIO.HIGH)
            GPIO.output(gpio_pin_l, GPIO.HIGH)
            p_r.ChangeDutyCycle(40)
	    p_l.ChangeDutyCycle(40)
            time.sleep(3)

            p_r.stop()
	    p_l.stop()
	    # lcd.lcd_display_string("Stop")
            print("stop")

            p_r.start(0)
	    p_l.start(0)
            time.sleep(1)
            GPIO.output(gpio_pin_r, GPIO.LOW)
            GPIO.output(gpio_pin_l, GPIO.LOW)
            time.sleep(1)
	    # lcd.lcd_display_string("Low ")
            print("gpio low")
            p_r.ChangeDutyCycle(40)
	    p_l.ChangeDutyCycle(40)
            time.sleep(2)
    finally:
        p_r.stop()
	p_l.stop()
        GPIO.cleanup()

if __name__ == '__main__':
    main()
    GPIO.cleanup()
