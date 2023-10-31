#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import time
import serial
import spidev
import RPi.GPIO as GPIO
import pigpio
import lcd_display
from config import Pin, PWM, FOLLOWPID, HumanFollowParam, Control
from motor import Motor
# ros
from sensor_msgs.msg import Joy
from tang_msgs.msg import HumanInfo, Modechange, IsDismiss, Emergency, ObstacleDistance
from geometry_msgs.msg import Twist

# spi settings
spi = spidev.SpiDev()
spi.open(0,0)
spi.max_speed_hz = 100000 

GPIO.setmode(GPIO.BCM)

# atomlite setting
# usb_device = serial.Serial('/dev/ttyUSB0', '115200', timeout=1.0)
# atom_command = 'hearton'
# usb_device.write(atom_command.encode())

class TangController():
    def __init__(self):
        self.pi = pigpio.pi()
        # モードのGPIOピン
        GPIO.setup(Pin.teleop_mode, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(Pin.follow_mode, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.add_event_detect(Pin.teleop_mode, GPIO.FALLING, callback=self.switch_on_callback, bouncetime=10)
        GPIO.add_event_detect(Pin.follow_mode, GPIO.FALLING, callback=self.switch_on_callback, bouncetime=10)
        # HumanInfo.msg
        self.human_info = HumanInfo()
        self.cmdvel_from_imu = Twist()
        # Modechange.msg
        self.current_mode = Modechange()
        self.current_mode.realsense_depth_thresh = 4.0
        self.main = 0
        self.ref_pos = 0.0
        self.speed = 0.6
        self.prev_command = 0
        self.command_pwm = 0
        # LCD Display
        self.mylcd = lcd_display.lcd()
        # motor
        self.motor = Motor()
        self.prev_linear_velocity = 0
        self.prev_angular_velocity = 0
        self.emergency_prev_btn = None

        # subscribe to motor messages on topic "tang_cmd", 追跡対象の位置と大きさ
        self.human_info_sub = rospy.Subscriber('tang_cmd', HumanInfo, self.cmd_callback, queue_size=1)
        self.imu_sub = rospy.Subscriber('cmdvel_from_imu', Twist, self.imu_callback, queue_size=1)
        self.is_dismiss_sub = rospy.Subscriber('is_dismiss', IsDismiss, self.dismiss_callback, queue_size=1)
        self.is_dismiss = IsDismiss()
        self.is_emergency_sub = rospy.Subscriber('emergency', Emergency, self.emergency_callback, queue_size=1)
        self.is_emergency = Emergency()
        # 近接物体があるかないか
        self.obstacle_distance_sub = rospy.Subscriber('obstacle_distance', ObstacleDistance, self.obstacle_distance_callback, queue_size=1)
        self.is_obstacle = False
        # publisher, モードと距離の閾値、赤色検出の閾値をpub
        self.mode_pub = rospy.Publisher('current_param', Modechange, queue_size=1)
        self.prev_mode = 0
        # ローパスフィルタ
        self.alpha = 0.1
    
    def read_analog_pin(self, channel):
        adc = spi.xfer2([1, (8 + channel)<<4, 0])
        data = ((adc[1]&3) << 8) + adc[2]
        return data
    
    def switch_on_callback(self, gpio):
        result = GPIO.input(gpio) # ピンの値を読み取る(HIGH or LOWの1 or 0)
        atom_command = ''
        if(gpio == Pin.teleop_mode and result == 0): 
            self.main = 0
            atom_command = 'manual'
            print("Manual",gpio)
        if(gpio == Pin.follow_mode and result == 0): 
            self.main = 1
            atom_command = 'human'
            print("-------------------------Human",gpio)
        # usb_device.write(atom_command.encode())
        self.current_mode.mode = self.main
        self.mode_pub.publish(self.current_mode)
        return
    
    def emergency_callback(self, msg):
        if(msg.is_emergency == True):
            print("緊急停止モード:")
            self.stop_control()
            self.main = 99
            self.emergency_prev_btn = True
        elif(self.emergency_prev_btn == True and msg.is_emergency == False):
            self.main = self.prev_mode
            self.emergency_prev_btn = False
        return

    def obstacle_distance_callback(self, msg):
        if(0.0 < msg.distance.data < 0.3):
            print(f"障害物検出: {msg.distance.data}")
            self.is_obstacle = True
            self.stop_control()
            self.main = 99
        else:
            self.main = self.prev_mode
            self.is_obstacle = False
        return

    def cmd_callback(self, msg):
        # 人の位置とサイズを得る
        self.human_info = msg
        return
    
    def imu_callback(self, msg):
        self.cmdvel_from_imu = msg
        return

    def dismiss_callback(self, msg):
        self.is_dismiss = msg
        return

    def change_velocity(self, cnt):
        if(self.speed == Control.max_linear_vel_manual):
            self.speed = 0.3
            cnt = 0
            return cnt
        swt_val = self.read_analog_pin(Pin.swt_channel)
        if(swt_val == 0):
            cnt += 1
            if(cnt > 30):
                self.speed += 0.1
                self.mylcd.lcd_display_string("Speed: " + str(self.speed), 2)
                cnt = 0
        else:
            cnt = 0
        Control.max_linear_vel = self.speed
        return cnt
    
    def lowpass_filter(self, input_value, prev_value):
        if prev_value is None:
            prev_value = input_value
        output = self.alpha*input_value + (1-self.alpha)*prev_value
        return output
    
    def nomarilze_speed(self, motor_r, motor_l):
        if(motor_r >= 100): motor_r = 100
        if(motor_r <= -100): motor_r = -100
        if(motor_l >= 100): motor_l = 100
        if(motor_l <= -100): motor_l = -100
        return abs(motor_r), abs(motor_l)

    def p_control(self, cur_pos):
        """
        @fn p_control()
        @details P制御
        """
        current_command = FOLLOWPID.p_gain * (self.ref_pos - cur_pos) + FOLLOWPID.d_gain*((self.ref_pos - cur_pos) - self.prev_command)/FOLLOWPID.dt
        self.prev_command = self.ref_pos - cur_pos
        print(f"current_command: {cur_pos}")
        return current_command

    def send_vel_cmd(self, r_duty, l_duty):
        # PWMを出力
        r_duty, l_duty = self.nomarilze_speed(r_duty, l_duty)
        r_cnv_dutycycle = int((r_duty * 1000000 / 100))
        l_cnv_dutycycle = int((l_duty * 1000000 / 100))
        self.pi.hardware_PWM(Pin.pwm_r, PWM.freq, r_cnv_dutycycle)
        self.pi.hardware_PWM(Pin.pwm_l, PWM.freq, l_cnv_dutycycle)
        # rospy.loginfo("r_duty : %d | l_duty: %d", r_duty, l_duty)
        return

    def manual_control(self):
        # Read the joystick position data
        vrx_pos = self.read_analog_pin(Pin.vrx_channel) / Control.max_joystick_val * 2 - 1  # normalize to [-1, 1]
        vry_pos = self.read_analog_pin(Pin.vry_channel) / Control.max_joystick_val * 2 - 1  # normalize to [-1, 1]
        # Debugging
        print("Normalized X : {}  Normalized Y : {} ".format(vrx_pos, vry_pos))
        # Calculate linear and angular velocity
        linear_velocity = Control.max_linear_vel * max(vry_pos, 0)  # vry_pos negative would mean backward, but we restrict that
        angular_velocity = vrx_pos * Control.max_angular_vel
        # Set velocities to zero if they are below the threshold
        if abs(linear_velocity) < Control.velocity_thresh:
            linear_velocity = 0
        if abs(angular_velocity) < Control.velocity_thresh:
            angular_velocity = 0
        print(f"linear_velocity{linear_velocity}, angular_velocity{angular_velocity}")
        # Decide acceleration for linear velocity
        linear_acceleration = Control.a_target
        if linear_velocity == 0:
            linear_acceleration = Control.d_target
        # Decide acceleration for angular velocity
        if angular_velocity < self.prev_angular_velocity:
            angular_acceleration = -Control.alpha_target
        else:
            angular_acceleration = Control.alpha_target
        self.motor.run(linear_velocity, angular_velocity, linear_acceleration, angular_acceleration)
        # Update previous velocities
        self.prev_linear_velocity = linear_velocity
        self.prev_angular_velocity = angular_velocity
        return

    def follow_control(self):
        # 速度を距離に従って減衰させる、1m20cm以内で減衰開始する
        if self.is_dismiss.flag: 
            self.send_vel_cmd(0, 0)
            return
        if(self.human_info.human_point.x >= 2.0 or self.human_info.human_point.x < 0.0):
            command_depth = 1.0
        else:
            command_depth = self.human_info.human_point.x / 2.0
        linear_velocity = Control.max_linear_vel * command_depth
        # コマンドの制御量を比例制御で決める
        angular_velocity = -self.p_control(self.human_info.human_point.y)
        # 80cm以内であれば止まる
        if self.human_info.human_point.x <= HumanFollowParam.depth_min_thresh and self.human_info.human_point.x > 0.0:
            rospy.logwarn("Stop: %lf", self.human_info.human_point.x)
            linear_velocity = 0.0
            angular_velocity = 0.0
        # 加速度設定
        linear_acceleration = Control.a_target
        if linear_velocity == 0:
            linear_acceleration = Control.d_target
        # 減速時の加速度設定
        if angular_velocity < self.prev_angular_velocity:
            angular_acceleration = -Control.alpha_target
        else:
            angular_acceleration = Control.alpha_target
        # ローパスフィルタ
        linear_velocity = self.lowpass_filter(linear_velocity, self.prev_linear_velocity)
        angular_velocity = self.lowpass_filter(angular_velocity, self.prev_angular_velocity)
        # モータに指令値を送る
        self.motor.run(linear_velocity, angular_velocity, linear_acceleration, angular_acceleration)
        # Update previous velocities
        self.prev_linear_velocity = linear_velocity
        self.prev_angular_velocity = angular_velocity
        return
    
    def change_control_mode(self):
        if self.main == 99: return
        self.prev_mode = self.main
        if self.main == 0:
            self.manual_control()
        elif self.main == 1:
            self.follow_control()
        return
    
    def stop_control(self):
        self.motor.error_sum = {'v' : 0, 'w' : 0}
        self.motor.prev_error = {'v' : 0, 'w' : 0}
        self.encoder_values = {'r' : 0, 'l' : 0}
        self.prev_encoder_values = {'r' : 0, 'l' : 0}
        return
            
def main():
    rospy.init_node("tang_control", anonymous=True)
    rate = rospy.Rate(10)
    tang_controller = TangController()
    cnt = 0
    tang_controller.mylcd.lcd_display_string("Start Control", 1)
    tang_controller.mylcd.lcd_display_string("Speed: " + str(tang_controller.speed), 2)
    while not rospy.is_shutdown():
        cnt = tang_controller.change_velocity(cnt)
        tang_controller.change_control_mode()
        rate.sleep()
    tang_controller.send_vel_cmd(0, 0)
    tang_controller.pi.stop()
    tang_controller.motor.stop()	
    GPIO.cleanup()
    rospy.spin()
                
if __name__ == '__main__':
    main()
    atom_command = 'heartoff'
    # usb_device.write(atom_command.encode())
