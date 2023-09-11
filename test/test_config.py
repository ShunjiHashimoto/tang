#!/usr/bin/env python3
# -*- coding: utf-8 -*-

class Pin:
    encoder_r_A  = 5
    encoder_r_B  = 6
    direction_r = 18
    pwm_r        = 13 
    encoder_l_A  = 22
    encoder_l_B  = 27
    direction_l = 17
    pwm_l        = 12

class PID:
    Kp = 1.2
    Kd = 0.0001
    Ki = 0.01
    dt = 0.001 # 0.0001がmax
    
class PWM:
    # PWM周波数をHzで指定
    freq = 1000 # [Hz]
    max_duty = 70
    
class Fig:
    time_data  = []
    vel_data_r = []
    vel_data_l = []
    target_vel_data = []

class Control:
    # 入力電圧
    input_v = 12
    # 目標角速度
    w_target = 0.0001
    # 目標速度
    v_target = 1.0
    # 逆起電圧定数
    Ke_r = 1.5295
    Ke_l = 1.4848
    # トルク定数
    Kt_r = 0
    Kt_l = 0
    # 巻線抵抗
    R = 0
    # エンコーダ値1あたりの回転角度[rad]
    radian_1encoder_r = 0.00306
    radian_1encoder_l = 0.003095
    # モータ１回転あたりのエンコーダ値
    encoder_1rotation_r = 2030
    encoder_1rotation_l = 2050
    # モータの回転数
    rotation_num = 2
    # トレッド幅[m]
    tread_w = 0.32
    # 車輪半径
    wheel_r = 0.06553/2

    