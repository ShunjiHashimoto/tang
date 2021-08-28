#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import cv2
import sys # sysはPythonのインタプリタや実行環境に関する情報を扱うためのライブラリです。
import numpy as np

file_path = '/home/hashimoto/Videos/capture/human.mp4'
delay = 1
window_name = 'frame'

video = cv2.VideoCapture(0)

if not video.isOpened():
    sys.exit()

while True:
    ret, frame = video.read() # カメラの画像を１フレーム読み込み、frameに格納、retは読み込めたらtrueを格納する
    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # HoG特徴量の計算
    hog = cv2.HOGDescriptor()
    # サポートベクタマシンによる人検出
    hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
    hogParams = {}
 
    # 人を検出した座標
    human, r = hog.detectMultiScale(frame_gray, **hogParams)

    # バウンディングボックス
    if(len(human) > 0):
        for (x, y, w, h) in human:
            cv2.rectangle(frame, (x, y),(x+w, y+h),(0,50,255), 3)
 
    if ret:
        cv2.imshow(window_name, frame)
        if cv2.waitKey(delay) & 0xFF == ord('q'): # qが押されると終了
            break
    else:
        video.set(cv2.CAP_PROP_POS_FRAMES, 0)

cv2.destroyWindow(window_name)
video.release()