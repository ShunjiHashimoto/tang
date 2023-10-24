#!/usr/bin/env python3
# -*- coding: utf-8 -*-

class CameraParam:
    # カメラ画像の横幅を1280に設定
    width = 1280
     # カメラ画像の縦幅を720に設定
    height = 720
    # カメラFPSを30FPSに設定
    fps = 30
    # 画面に表示するカメラ画像の名前
    window_name = 'color object detection'
    
class ColorObjectParam:
    # 色相
    hue_min = 40
    hue_max = 80
    # 彩度
    sat_min = 0
    sat_max = 127
    # 明度
    val_min = 0
    val_max = 255
    # 追跡対象としてみなす領域の最小値
    object_min_area = 100
    
class HumanFollowParam:
    # 対象を見失ってから停止指令を送るまでの時間
    dismiss_time = 0.5
