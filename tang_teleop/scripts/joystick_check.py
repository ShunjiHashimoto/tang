#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import sys
import time
import RPi.GPIO as GPIO
import spidev

# modeを選択
GPIO.setmode(GPIO.BCM)

gpio_pin_r = 8
gpio_pin_l = 17
teleop_mode_gpio = 5
follow_mode_gpio = 6
# デジタル出力ピンを設定, 回転方向を決められる
# DIG1 = 11(LEFT), DIG2 = 12(RIGHT)
GPIO.setup(gpio_pin_r, GPIO.OUT)
GPIO.setup(gpio_pin_l, GPIO.OUT)

output_pin_r = 13
output_pin_l = 12
# アナログ出力ピンを設定、output_pinを32,33に設定
# ANA1 = 32(LEFT), ANA2 = 33(RIGHT)
GPIO.setup(output_pin_r, GPIO.OUT)
GPIO.setup(output_pin_l, GPIO.OUT)
GPIO.output(output_pin_r, GPIO.LOW)
GPIO.output(output_pin_l, GPIO.LOW)
# PWMサイクルを50Hzに設定
p_r = GPIO.PWM(output_pin_r, 50)
p_l = GPIO.PWM(output_pin_l, 50)

p_r.start(0)
p_l.start(0)

BTN_BACK = 0x0100
BTN_Y = 0x0001
BTN_A = 0x0002
AXS_MAX = 1.0
AXS_OFF = 0.0
GPIO.setup(teleop_mode_gpio, GPIO.IN)
GPIO.setup(follow_mode_gpio, GPIO.IN)


# spi settings
spi = spidev.SpiDev()
spi.open(0,0)
spi.max_speed_hz = 1000000 
swt_channel = 2
vrx_channel = 0
vry_channel = 1

def read_channel(channel):
    adc = spi.xfer2([1, (8 + channel)<<4, 0])
    data = ((adc[1]&3) << 8) + adc[2]
    return data

def change_move_speed(cnt):
    swt_val = read_channel(swt_channel)
    print("switch : {} ".format(swt_val))
    return cnt

def main():
    cnt = 0
    while(1):
        time.sleep(0.1)
        cnt = change_move_speed(cnt)
        vrx_pos = read_channel(vrx_channel)
        vry_pos = read_channel(vry_channel)
        # Read switch state
        print("X_flat : {}  Y_verti : {} ".format(vrx_pos, vry_pos))
    
    # modeを選択
    # GPIO.setmode(GPIO.BCM)
    # gpio_pin_l = 18
    # gpio_pin_r = 17
    # GPIO.setup(gpio_pin_r, GPIO.OUT)
    # GPIO.setup(gpio_pin_l, GPIO.OUT)
    # GPIO.setup(11, GPIO.OUT, initial = GPIO.LOW)
    # GPIO.setup(19, GPIO.OUT, initial = GPIO.LOW)
    # GPIO.setup(21, GPIO.OUT, initial = GPIO.LOW)
    # GPIO.setup(16, GPIO.OUT, initial = GPIO.LOW)

    # # アナログピン, ANA2(RIGHT) = 32
    # pwm_pin_r = 12
    # GPIO.setup(pwm_pin_r, GPIO.OUT)
    # p_r = GPIO.PWM(pwm_pin_r, 40)
    # p_r.start(0)

    # pwm_pin_l = 13
    # GPIO.setup(pwm_pin_l, GPIO.OUT)
    # p_l = GPIO.PWM(pwm_pin_l, 40)
    # p_l.start(0)
    # # lcd = lcd_display.lcd()
    # try:
    #     while (1):
    #         time.sleep(1)
    #         # デジタル出力ピンを設定, 回転方向を決められる
    #         # DIG1 = 27(LEFT), DIG2 = 18(RIGHT)
    #         print("gpio high")
	#     # lcd.lcd_display_string("High")
    #         GPIO.output(gpio_pin_r, GPIO.HIGH)
    #         GPIO.output(gpio_pin_l, GPIO.HIGH)
    #         p_r.ChangeDutyCycle(40)
	#     p_l.ChangeDutyCycle(40)
    #         time.sleep(3)

    #         p_r.stop()
	#     p_l.stop()
	#     # lcd.lcd_display_string("Stop")
    #         print("stop")

    #         p_r.start(0)
	#     p_l.start(0)
    #         time.sleep(1)
    #         GPIO.output(gpio_pin_r, GPIO.LOW)
    #         GPIO.output(gpio_pin_l, GPIO.LOW)
    #         time.sleep(1)
	#     # lcd.lcd_display_string("Low ")
    #         print("gpio low")
    #         p_r.ChangeDutyCycle(40)
	#     p_l.ChangeDutyCycle(40)
    #         time.sleep(2)
    # finally:
    #     p_r.stop()
	# p_l.stop()
    #     GPIO.cleanup()

if __name__ == '__main__':
    main()
    GPIO.cleanup()
