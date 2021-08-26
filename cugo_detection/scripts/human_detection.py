#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import cv2
import sys # sysはPythonのインタプリタや実行環境に関する情報を扱うためのライブラリです。
import numpy as np

file_path = '/home/hashimoto/Videos/capture/opencv.mp4'
delay = 1
window_name = 'frame'

fullbody_detector = cv2.CascadeClassifier("/usr/share/opencv/haarcascades/haarcascade_fullbody.xml")
video = cv2.VideoCapture(0)

if not video.isOpened():
    sys.exit()

# params for ShiTomasi corner detection
feature_params = dict( maxCorners = 100,
                       qualityLevel = 0.3,
                       minDistance = 7,
                       blockSize = 7 )
# Parameters for lucas kanade optical flow
lk_params = dict( winSize  = (15,15),
                  maxLevel = 2,
                  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
# Create some random colors
color = np.random.randint(0,255,(100,3))

# Take first frame and find corners in it
ret, old_frame = video.read()
old_gray = cv2.cvtColor(old_frame, cv2.COLOR_BGR2GRAY)
# コーナー検出(Harris + ShiTomasi)
p0 = cv2.goodFeaturesToTrack(old_gray, mask = None, **feature_params)
# Create a mask image for drawing purposes
mask = np.zeros_like(old_frame)

while True:
    ret, frame = video.read() # カメラの画像を１フレーム読み込み、frameに格納、retは読み込めたらtrueを格納する
    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # calculate optical flow
    body = fullbody_detector.detectMultiScale(frame_gray,scaleFactor=1.1, minNeighbors=3, minSize=(40, 40))
    # frame = cv2.resize(frame, (int(frame.shape[1]/3), int(frame.shape[0]/3))) # 1/4にリサイズ, shape[1]が高さ, shape[0]が幅
    # 画像の色成分の分割と統合
    # b, g, r = cv2.split(frame)
    # g = frame[:,:,0]
    # calculate optical flow
    p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)
    # オプティカルフローを検出した特徴点を選別（0：検出せず、1：検出した）
    good_new = p1[st == 1]
    good_old = p0[st == 1]
    # if(len(good_new) == 0 or len(good_old) == 0): break

    # オプティカルフローを描画
    for i,(new,old) in enumerate(zip(good_new,good_old)):
        a,b = new.ravel()
        c,d = old.ravel()
        mask = cv2.line(mask, (int(a),int(b)),(int(c),int(d)), color[i].tolist(), 2)
        frame = cv2.circle(frame,(int(a),int(b)),5,color[i].tolist(),-1)
    img = cv2.add(frame,mask)

    # 人検出した部分を長方形で囲う
    for (x, y, w, h) in body:
        cv2.rectangle(img, (x, y),(x+w, y+h),(0,255,0),2)

    if ret:
        cv2.imshow(window_name, img)
        if cv2.waitKey(delay) & 0xFF == ord('q'): # qが押されると終了
            break
        # Now update the previous frame and previous points
        old_gray = frame_gray.copy()
        p0 = good_new.reshape(-1,1,2)
    else:
        video.set(cv2.CAP_PROP_POS_FRAMES, 0)

cv2.destroyWindow(window_name)
video.release()
