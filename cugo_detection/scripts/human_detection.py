#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import sys # sysはPythonのインタプリタや実行環境に関する情報を扱うためのライブラリです。

file_path = '/home/hashimoto/Videos/capture/opencv.mp4'
delay = 1
window_name = 'frame'

video = cv2.VideoCapture(file_path)

if not video.isOpened():
    sys.exit()

while True:
    ret, frame = video.read() # カメラの画像を１フレーム読み込み、frameに格納、retは読み込めたらtrueを格納する
    frame = cv2.resize(frame, (int(frame.shape[1]/3), int(frame.shape[0]/3))) # 1/4にリサイズ, shape[1]が高さ, shape[0]が幅
    # 画像の色成分の分割と統合
    b, g, r = cv2.split(frame)
    # g = frame[:,:,0]

    if ret:
        cv2.imshow(window_name, b)
        if cv2.waitKey(delay) & 0xFF == ord('q'): # qが押されると終了
            break
    else:
        video.set(cv2.CAP_PROP_POS_FRAMES, 0)

cv2.destroyWindow(window_name)
