#!/usr/bin/env python3
# -*- coding: utf-8 -*-

class Pin:
    encoder_l_A  = 5
    encoder_l_B  = 6
    direction_l = 18
    pwm_l        = 13 
    encoder_r_A  = 22
    encoder_r_B  = 27
    direction_r = 17
    pwm_r        = 12
    emergency_mode = 16

class FOLLOWPID:
    p_gain = 0.002
    d_gain = 0.0
    #d_gain = 7.0
    dt = 0.01

class PID:
    Kp = 1.0
    Ki = 0.01
    Kd = 0.00
    dt = 0.005 # 0.0001がmax
    max_error_sum_v = 2
    max_error_sum_w = 2
    
class PWM:
    # PWM周波数をHzで指定
    freq = 1000 # [Hz]
    max_duty = 50
    
class Fig:
    time_data  = []
    vel_data = []
    w_data = []
    target_vel_data = []
    target_w_data = []
    target_a_data = []

class Control:
    # 入力電圧
    input_v = 24
    # 最大角速度
    max_w = 0.8
    # 最大速度
    max_velocity = 0.8
    # 目標角速度
    w_target = 0.2
    # 目標速度
    v_target = 0.4
    # 目標加速度
    a_target = 0.01
    # 目標減速加速度
    d_target = -0.1
    # 目標角加速度
    alpha_target = 0.01
    # 逆起電圧定数
    Ke_r = 0.5905
    Ke_l = 0.5905
    # トルク定数
    Kt_r = 0.6666
    Kt_l = 0.6666
    # 巻線抵抗
    R = 3.05931
    # エンコーダ値1あたりの回転角度[rad]
    radian_1encoder_r = 0.00306
    radian_1encoder_l = 0.003095
    # モータ１回転あたりのエンコーダ値
    encoder_1rotation_r = 2030
    encoder_1rotation_l = 2030
    # モータの回転数
    rotation_num = 5
    # トレッド幅[m]
    tread_w = 0.32
    # 車輪半径[m]
    wheel_r = 0.05
    # 車体質量
    M = 24.6
    # 車体慣性モーメント J = ml^2
    # J = 1/3(a^2 + b^2) 44cm, 40cm = 0.58999
    J = M*(0.22*0.22 + 0.2*0.2)/3
    
class HumanFollowParam:
    # 対象物が小さい場合は追従しないためのパラメータ
    min_area_thresh = 40
    # 対象物の大きさに応じて速度を変更するためのパラーメタ
    max_area_thresh = 300
    # 画像中心のx座標
    image_center = 1280/2
