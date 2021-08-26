#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################################################################################
# opencv tutorial shi-tomasiコーナー検出
###########################################################################################################################
import cv2
import sys # sysはPythonのインタプリタや実行環境に関する情報を扱うためのライブラリです。
import numpy as np

file_path = '/home/hashimoto/Videos/capture/opencv.mp4'
delay = 1
window_name = 'frame'
maxcorners= 100  # 検出個数
quality = 0.1   # 品質(0～1), 固有値の最小値のしきい値，0.1 or 0.01
distance = 10   # ユークリッド距離

video = cv2.VideoCapture(file_path)

if not video.isOpened():
    sys.exit()

while True:
    ret, frame = video.read() # カメラの画像を１フレーム読み込み、frameに格納、retは読み込めたらtrueを格納する
    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # モノクロ化
    corners = cv2.goodFeaturesToTrack(frame_gray, maxcorners, quality, distance) # goodFeaturesToTrack(画像, 検出個数, 品質, ユークリッド距離)
    corners = np.int0(corners)

    if ret:
        print(len(corners))
        for i in corners:
          x,y = i.ravel() # 行列の１次元化，numpyのravel
          cv2.circle(frame, (x,y), 3, 255, -1) # circle(描画先の画像, 円の中心の位置, 円の半径, 色, 線の太さ)
    else:
        video.set(cv2.CAP_PROP_POS_FRAMES, 0)

    frame = cv2.resize(frame, (int(frame.shape[1]/3), int(frame.shape[0]/3))) # 1/4にリサイズ, shape[1]が高さ, shape[0]が幅
    cv2.imshow(window_name, frame)
    if cv2.waitKey(delay) & 0xFF == ord('q'): # qが押されると終了
        break

cv2.destroyWindow(window_name)
