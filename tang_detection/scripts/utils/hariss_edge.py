#!/usr/bin/env python3
# -*- coding: utf-8 -*-
###########################################################################################################################
# opencv tutorial Harissコーナー検出
# https://whitewell.sakura.ne.jp/OpenCV/py_tutorials/py_feature2d/py_features_harris/py_features_harris.html#harris-corners
# 概要
# 画像の画素の位置(u, v)をいろいろな方向に動かしてみて，画素値がどのように変化するか求める．
# 画素値の変化量が大きい場合はコーナーと判定できる．
###########################################################################################################################
import cv2
import sys # sysはPythonのインタプリタや実行環境に関する情報を扱うためのライブラリです。
import numpy as np

file_path = '/home/hashimoto/Videos/capture/opencv.mp4'
delay = 1
window_name = 'frame'

video = cv2.VideoCapture(file_path)

if not video.isOpened():
    sys.exit()

while True:
    ret, frame = video.read() # カメラの画像を１フレーム読み込み、frameに格納、retは読み込めたらtrueを格納する
    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # モノクロ化
    frame_gray = np.float32(frame_gray)
    dst = cv2.cornerHarris(frame_gray, 2,3, 0.04)
    
    # dst = cv2.dilate(dst, None) # ターゲットの大きさが1pxより広がり，コーナー判定箇所が増える

    frame[dst>0.01*dst.max()] = [0, 0, 255]

    frame = cv2.resize(frame, (int(frame.shape[1]/3), int(frame.shape[0]/3))) # 1/4にリサイズ, shape[1]が高さ, shape[0]が幅

    if ret:
        cv2.imshow(window_name, frame)
        if cv2.waitKey(delay) & 0xFF == ord('q'): # qが押されると終了
            break
    else:
        video.set(cv2.CAP_PROP_POS_FRAMES, 0)

cv2.destroyWindow(window_name)
