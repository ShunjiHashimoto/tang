#!/usr/bin/env python
# -*- coding: utf-8 -*-

## just teleop tang ##
import rospy
import time
import RPi.GPIO as GPIO
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

# circle pitch
Pitch = 6.1850105367549055/2

def add_right_gear_count(channel):
    cnt_list[0]+=1

def add_left_gear_count(channel):
    cnt_list[1]+=1

def calc_velocity(count, time):
    return Pitch*count/time

BTN_BACK = 0x0100
BTN_Y = 0x0001
BTN_A = 0x0002
AXS_MAX = 1.0
AXS_OFF = 0.0

cnt_list = [0, 0]

# modeを選択
GPIO.setmode(GPIO.BCM)

# gpio_pin_r = 18
# gpio_pin_l = 17
# # デジタル出力ピンを設定, 回転方向を決められる
# # DIG1 = 11(LEFT), DIG2 = 12(RIGHT)
# GPIO.setup(gpio_pin_r, GPIO.OUT)
# GPIO.setup(gpio_pin_l, GPIO.OUT)

# output_pin_r = 12
# output_pin_l = 13
# # アナログ出力ピンを設定、output_pinを32,33に設定
# # ANA1 = 32(LEFT), ANA2 = 33(RIGHT)
# GPIO.setup(output_pin_r, GPIO.OUT)
# GPIO.setup(output_pin_l, GPIO.OUT)
# GPIO.output(output_pin_r, GPIO.LOW)
# GPIO.output(output_pin_l, GPIO.LOW)
# # PWMサイクルを50Hzに設定
# p_r = GPIO.PWM(output_pin_r, 50)
# p_l = GPIO.PWM(output_pin_l, 50)
# p_r.start(0)
# p_l.start(0)

# 割り込みイベント設定
right_gear_pin = 23
left_gear_pin = 27
GPIO.setup(right_gear_pin, GPIO.IN)
GPIO.setup(left_gear_pin, GPIO.IN)
# bouncetimeは割り込みを行った後設定された時間は割り込みを検知しないという糸
GPIO.add_event_detect(right_gear_pin, GPIO.BOTH, bouncetime=1)
GPIO.add_event_detect(left_gear_pin, GPIO.BOTH, bouncetime=1)
# コールバック関数登録
GPIO.add_event_callback(right_gear_pin, add_right_gear_count) 
GPIO.add_event_callback(left_gear_pin, add_left_gear_count) 

class TangController():
    def __init__(self):
        self.btn = self.joy_l = self.joy_r = 0
        self.main = 0
        self.speed = rospy.get_param("/tang_teleop/speed")
        self.dt = 0.1
        self.d = 0.0 # distance 
        self.joy_sub = rospy.Subscriber("joy", Joy, self.joy_callback, queue_size=1)
        self.prev_time = 0.0
        self.count_sum = 0
    
        
    def mode_change(self):
        now = time.time()
        self.dt = now - self.prev_time
        self.prev_time = now
        if rospy.is_shutdown(): return
        # if self.d > 1000: 
        #     p_r.start(0)
        #     p_l.start(0)
        #     return
        if self.main == 0:
            # motor_l = self.joy_l
            # motor_r = self.joy_r
            vel_r = calc_velocity(cnt_list[0], self.dt )
            vel_l = calc_velocity(cnt_list[1], self.dt )
            v = (vel_l+vel_r)/2
            print(cnt_list)
            self.count_sum += cnt_list[0]
            print("distance", self.count_sum*Pitch)
            cnt_list[0] = cnt_list[1] = 0
            self.d += v*self.dt

            print("distance: %d, velocity: %lf", self.d, v)
            print("delta_time", self.dt)
            # if motor_l >= 0 and motor_r >= 0:
            #     GPIO.output(gpio_pin_r, GPIO.HIGH)
            #     GPIO.output(gpio_pin_l, GPIO.HIGH)
            #     p_r.ChangeDutyCycle(motor_r)
            #     p_l.ChangeDutyCycle(motor_l)
            #     # rospy.loginfo("Go! | motor_l : %d | motor_r: %d", motor_l, motor_r)
            # elif motor_l < 0 and motor_r < 0:
            #     GPIO.output(gpio_pin_r, GPIO.LOW)
            #     GPIO.output(gpio_pin_l, GPIO.LOW)
            #     p_r.ChangeDutyCycle(-(motor_r))
            #     p_l.ChangeDutyCycle(-(motor_l))
                # rospy.loginfo("Back! | motor_l : %d | motor_r: %d", motor_l, motor_r)
            return

        else:
            print("object_detection mode")
        
    def joy_callback(self, joy_msg):
        # button[5]で上がる、button[4]で下がる、realsenseの認識距離変更
        newbtn = 0

        if(joy_msg.buttons[6]):
            newbtn |= BTN_BACK
        elif(joy_msg.buttons[0]):
            newbtn |= BTN_A
        elif(joy_msg.buttons[3]):
            newbtn |= BTN_Y
        
        joy_l = joy_msg.axes[1]
        if(joy_l <= -AXS_OFF):
            joy_l += AXS_OFF
        elif(joy_l >= AXS_OFF):
            joy_l -= AXS_OFF
        else:
            joy_l = 0
            
        joy_r = joy_msg.axes[3]
        if(joy_r <= -AXS_OFF):
            joy_r += AXS_OFF
        elif(joy_r >= AXS_OFF):
            joy_r -= AXS_OFF
        else:
            joy_r = 0
        
        self.joy_l = int(self.speed * joy_l / (AXS_MAX - AXS_OFF))
        self.joy_r = int(self.speed * joy_r / (AXS_MAX - AXS_OFF))
            
        push = ((~self.btn) & newbtn)
        self.btn = newbtn
        if(push & BTN_BACK):
            self.main = (self.main + 1)%2
        elif(push & BTN_Y):
            self.speed += 10
        elif(push & BTN_A):
            self.speed -= 10
            
        if(self.speed > 100):
            self.speed = 100
        elif(self.speed < 10):
            self.speed = 10

def main():
    # start node
    rospy.init_node("tang_teleop", anonymous=True)
    instance = TangController()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        instance.mode_change()
        rate.sleep()
    rospy.spin()
                
                
if __name__ == '__main__':
    main()
    GPIO.cleanup()
