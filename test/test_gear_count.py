#!/usr/bin/env python
# -*- coding: utf-8 -*-
# import rospy
import sys
import time
import RPi.GPIO as GPIO
# circle pitch
Pitch = 6.1850105367549055
cnt_list = [0, 0]

def add_right_gear_count(channel):
    cnt_list[0]+=1

def add_left_gear_count(channel):
    cnt_list[1]+=1

def calc_velocity(count, time):
    return Pitch*count/time

def main():
    GPIO.setmode(GPIO.BCM)
    t = 0.01

    # 割り込みイベント設定
    right_gear_pin = 5
    left_gear_pin = 6
    GPIO.setup(right_gear_pin, GPIO.IN)
    GPIO.setup(left_gear_pin, GPIO.IN)
    # bouncetimeは割り込みを行った後設定された時間は割り込みを検知しないという糸
    GPIO.add_event_detect(right_gear_pin, GPIO.RISING, bouncetime=1)
    GPIO.add_event_detect(left_gear_pin, GPIO.RISING, bouncetime=1)
    # コールバック関数登録
    GPIO.add_event_callback(right_gear_pin, add_right_gear_count) 
    GPIO.add_event_callback(left_gear_pin, add_left_gear_count) 

    try:
        while (1):
            vel_r = calc_velocity(cnt_list[0], t)
            vel_l = calc_velocity(cnt_list[1], t)
            print( '\033[31m'+'右車輪のカウント数: '+'\033[0m'+str(cnt_list[0]))
            print( '\033[32m'+'左車輪のカウント数: '+'\033[0m'+str(cnt_list[1]))
            # cnt_list[0] = 0
            # cnt_list[1] = 0
            time.sleep(t)
    finally:
        GPIO.cleanup()

if __name__ == '__main__':
    main()
    GPIO.cleanup()
