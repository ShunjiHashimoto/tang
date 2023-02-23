#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import time
import RPi.GPIO as GPIO
from sensor_msgs.msg import Joy
from tang_msgs.msg import HumanInfo, Modechange
from geometry_msgs.msg import Twist
import spidev
import lcd_display
import serial

p_gain = 15.0 
d_gain = 7.0
#d_gain = 7.0

# modeを選択
GPIO.setmode(GPIO.BCM)

gpio_pin_r = 18
gpio_pin_l = 17
teleop_mode_gpio = 21
follow_mode_gpio = 16
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
spi.max_speed_hz = 100000 
swt_channel = 2
vrx_channel = 0
vry_channel = 1

# atomlite setting
usb_device = serial.Serial('/dev/ttyUSB0', '115200', timeout=1.0)
atom_command = 'hearton'
usb_device.write(atom_command.encode())

class TangController():
    def __init__(self):
        # HumanInfo.msg
        self.human_info = HumanInfo()
        self.cmdvel_from_imu = Twist()
        # Modechange.msg
        self.current_mode = Modechange()
        self.current_mode.realsense_depth_thresh = 4.0
        self.btn = self.joy_l = self.joy_r = 0
        self.main = 0
        self.ref_pos = 0.0
        self.speed = rospy.get_param("/tang_control/speed")
        self.prev_command = 0
        self.command_pwm = 0
        self.depth_min_thresh = 0.8
        self.dt = 0.1
        self.prev_joystick_switch = 1023
        # LCD Display
        self.mylcd = lcd_display.lcd()

        GPIO.add_event_detect(teleop_mode_gpio, GPIO.FALLING, callback=self.callback_switch_on, bouncetime=100)
        GPIO.add_event_detect(follow_mode_gpio, GPIO.FALLING, callback=self.callback_switch_on, bouncetime=100)

        # subscribe to motor messages on topic "tang_cmd", 追跡対象の位置と大きさ
        self.human_info_sub = rospy.Subscriber('tang_cmd', HumanInfo, self.cmd_callback, queue_size=1)
        self.imu_sub = rospy.Subscriber('cmdvel_from_imu', Twist, self.imu_callback, queue_size=1)
        # publisher, モードと距離の閾値、赤色検出の閾値をpub
        self.mode_pub = rospy.Publisher('current_param', Modechange, queue_size=1)
    
    def read_channel(self, channel):
        adc = spi.xfer2([1, (8 + channel)<<4, 0])
        data = ((adc[1]&3) << 8) + adc[2]
        return data
    
    # select mode
    def callback_switch_on(self, gpio):
        time.sleep(0.1)
        result = GPIO.input(gpio) # ピンの値を読み取る(HIGH or LOWの1 or 0)
        atom_command = ''
        if(gpio == teleop_mode_gpio and result == 0): 
            self.main = 0
            atom_command = 'manual'
        if(gpio == follow_mode_gpio and result == 0): 
            self.main = 1
            atom_command = 'human'
        usb_device.write(atom_command.encode())
        self.current_mode.mode = self.main
        self.mode_pub.publish(self.current_mode)
        print("mode:teleop=0, follow=1", self.main, "result", result, "gpio",gpio)
        return

    def change_move_speed(self, cnt):
        if(self.speed == 90):
            self.speed = 30
            cnt = 0
            return cnt
        swt_val = self.read_channel(swt_channel)
        print("switch : {} ".format(swt_val))
        if(swt_val > 1010):
            cnt += 1
            if(cnt > 30):
                self.speed += 10
                self.mylcd.lcd_display_string("Speed: " + str(self.speed), 2)
                cnt = 0
        else:
            cnt = 0
        return cnt
    
    def nomarilze_speed(self, motor_r, motor_l):
        if(motor_r >= 100): motor_r = 100
        if(motor_r <= -100): motor_r = -100
        if(motor_l >= 100): motor_l = 100
        if(motor_l <= -100): motor_l = -100
        return abs(motor_r), abs(motor_l)

    def mode_change(self):
        if self.main == 0:
            # Read the joystick position data
            vrx_pos = (self.read_channel(vrx_channel) -515)/8
            vry_pos = -(self.read_channel(vry_channel ) -515)/8
            # Read switch state
            print("X_flat : {}  Y_verti : {} ".format(vrx_pos, vry_pos))
            motor_r = 0
            motor_l = 0
            # 前進
            if(vry_pos > 1.0):
                # 左旋回
                if(vrx_pos < -1.0):
                    print("mode 1")
                    motor_r = vry_pos
                    motor_l = vry_pos + vrx_pos
                # 右旋回
                else:
                    print("mode 2")
                    motor_r = vry_pos - vrx_pos
                    motor_l = vry_pos 
                if(motor_l < 0): motor_l = 0
                if(motor_r < 0): motor_r = 0
                GPIO.output(gpio_pin_r, GPIO.HIGH)
                GPIO.output(gpio_pin_l, GPIO.HIGH)
            
            elif(vry_pos < -1.0):
                if(vrx_pos < -1.0):
                    print("mode 3")
                    motor_r = -vry_pos - vrx_pos
                    motor_l = -vry_pos 
                else:
                    print("mode 4")
                    motor_r = -vry_pos 
                    motor_l = -vry_pos + vrx_pos
                if(motor_l < 0): motor_l = 0
                if(motor_r < 0): motor_r = 0
                GPIO.output(gpio_pin_r, GPIO.HIGH)
                GPIO.output(gpio_pin_l, GPIO.HIGH)

            else:    
                print("Nothing") 
                
            motor_r, motor_l = self.nomarilze_speed(motor_r, motor_l)
            p_r.ChangeDutyCycle(motor_r)
            p_l.ChangeDutyCycle(motor_l)    
            
            # rospy.loginfo("motor_l : %d | motor_r: %d", motor_l, motor_r)
            return
        
        elif self.main == 1:
            # 速度を距離に従って減衰させる、1m20cm以内で減衰開始する
            if(self.human_info.human_point.x >= 2.0 or self.human_info.human_point.x < 0.0):
                command_depth = 1.0
            else:
                command_depth = self.human_info.human_point.x / 2.0
            motor_r = motor_l = self.speed * command_depth
            
            # コマンドの制御量を比例制御で決める
            self.command_pwm = self.p_control(self.human_info.human_point.y)

            if (abs(self.command_pwm) > motor_r and self.command_pwm < 0.0):
                self.command_pwm = -self.speed * command_depth
            elif (abs(self.command_pwm) > motor_r and self.command_pwm > 0.0):
                self.command_pwm = self.speed * command_depth
            # 80cm以内であれば止まる
            if self.human_info.human_point.x <= self.depth_min_thresh and self.human_info.human_point.x > 0.0:
                rospy.logwarn("Stop: %lf", self.human_info.human_point.x)
                motor_r = motor_l = 0
            elif (self.command_pwm < 0): # 右回り
                motor_r -= self.command_pwm/10
                motor_r = motor_r*1.1
                motor_l += self.command_pwm 
                # rospy.loginfo("motor_l %lf, motor_r %lf , | Turn Right", motor_l, motor_r)
            elif (self.command_pwm >= 0): # 左回り
                motor_l += self.command_pwm/10
                motor_l = motor_l*1.1
                motor_r -= self.command_pwm
                # rospy.loginfo("motor_l %lf, motor_r %lf , | Turn Left", motor_l, motor_r)
            GPIO.output(gpio_pin_r, GPIO.HIGH)
            GPIO.output(gpio_pin_l, GPIO.HIGH)
            try:
                p_r.ChangeDutyCycle(motor_r)
                p_l.ChangeDutyCycle(motor_l)
            except:
                rospy.logwarn("DutyCycle is over 100, %lf", self.command_pwm)
            return

            if(motor_r >= 100): motor_r = 100
            if(motor_r <= -100): motor_r = -100
            if(motor_l >= 100): motor_l = 100
            if(motor_l <= -100): motor_l = -100
            rospy.logwarn("motor_r:%d, motor_l:%d", motor_r, motor_l)
            # GPIO.output(gpio_pin_r, GPIO.HIGH)
            # GPIO.output(gpio_pin_l, GPIO.HIGH)
            p_r.ChangeDutyCycle(motor_r)
            p_l.ChangeDutyCycle(motor_l)
            return
        
    def p_control(self, cur_pos):
        """
        @fn p_control()
        @details P制御
        """
        current_command = p_gain * (self.ref_pos - cur_pos) + d_gain*((self.ref_pos - cur_pos) - self.prev_command)/self.dt
        self.prev_command = self.ref_pos - cur_pos
        #print("cur_pos: ", cur_pos, "diff between cur and ref: ", self.ref_pos - cur_pos)
        return current_command
    
    def cmd_callback(self, msg):
        # 人の位置とサイズを得る
        self.human_info = msg
    
    def imu_callback(self, msg):
        self.cmdvel_from_imu = msg
        
def main():
    rospy.init_node("tang_control", anonymous=True)
    instance = TangController()
    rate = rospy.Rate(10)
    cnt = 0
    instance.mylcd.lcd_display_string("Start Control", 1)
    instance.mylcd.lcd_display_string("Speed: " + str(instance.speed), 2)
    while not rospy.is_shutdown():
        cnt = instance.change_move_speed(cnt)
        instance.mode_change()
        rate.sleep()
    rospy.spin()
                
if __name__ == '__main__':
    main()
    atom_command = 'heartoff'
    usb_device.write(atom_command.encode())
    # GPIO.cleanup()
