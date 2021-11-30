#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import numpy as np
import sys # sysはPythonのインタプリタや実行環境に関する情報を扱うためのライブラリです。

# モデルの中の訓練されたクラス
classNames = {0: 'background',
              1: 'person', 2: 'bicycle', 3: 'car', 4: 'motorcycle', 5: 'airplane', 6: 'bus',
              7: 'train', 8: 'truck', 9: 'boat', 10: 'traffic light', 11: 'fire hydrant',
              13: 'stop sign', 14: 'parking meter', 15: 'bench', 16: 'bird', 17: 'cat',
              18: 'dog', 19: 'horse', 20: 'sheep', 21: 'cow', 22: 'elephant', 23: 'bear',
              24: 'zebra', 25: 'giraffe', 27: 'backpack', 28: 'umbrella', 31: 'handbag',
              32: 'tie', 33: 'suitcase', 34: 'frisbee', 35: 'skis', 36: 'snowboard',
              37: 'sports ball', 38: 'kite', 39: 'baseball bat', 40: 'baseball glove',
              41: 'skateboard', 42: 'surfboard', 43: 'tennis racket', 44: 'bottle',
              46: 'wine glass', 47: 'cup', 48: 'fork', 49: 'knife', 50: 'spoon',
              51: 'bowl', 52: 'banana', 53: 'apple', 54: 'sandwich', 55: 'orange',
              56: 'broccoli', 57: 'carrot', 58: 'hot dog', 59: 'pizza', 60: 'donut',
              61: 'cake', 62: 'chair', 63: 'couch', 64: 'potted plant', 65: 'bed',
              67: 'dining table', 70: 'toilet', 72: 'tv', 73: 'laptop', 74: 'mouse',
              75: 'remote', 76: 'keyboard', 77: 'cell phone', 78: 'microwave', 79: 'oven',
              80: 'toaster', 81: 'sink', 82: 'refrigerator', 84: 'book', 85: 'clock',
              86: 'vase', 87: 'scissors', 88: 'teddy bear', 89: 'hair drier', 90: 'toothbrush'}

file_path = '/home/hashimoto/Videos/capture/human.mp4'
video = cv2.VideoCapture(file_path)
window_name = 'frame'
delay = 1
 
# モデルの読み込み
model = cv2.dnn.readNetFromTensorflow('/home/hashimoto/catkin_ws/src/tang/tang_detection/models/frozen_inference_graph.pb',
                                      '/home/hashimoto/catkin_ws/src/tang/tang_detection/models/ssd_mobilenet_v2_coco_2018_03_29.pbtxt')

if not video.isOpened():
    sys.exit()

# テスト画像の読み込み
# image = cv2.imread("image.jpeg")
while True:
    ret, frame = video.read() # カメラの画像を１フレーム読み込み、frameに格納、retは読み込めたらtrueを格納する

    # 画像の縦と横サイズを取得
    image_height, image_width = frame.shape[:2]
 
    # Imageからblobに変換する
    model.setInput(cv2.dnn.blobFromImage(frame, size=(300, 300), swapRB=True))
 
    # 画像から物体検出を行う
    output = model.forward()
 
    # outputは[1:1:100:7]のリストになっているため、後半の2つを取り出す
    detections = output[0, 0, :, :]
 
    # detectionには[?,id番号、予測確率、Xの開始点、Yの開始点、Xの終了点、Yの終了点]が入っている。
    for detection in detections:
       # 予測確率を取り出し0.5以上か判定する。0.5以上であれば物体が正しく検出されたと判定する。
       confidence = detection[2]
       if confidence > .5:
           # id番号を取り出し、辞書からクラス名を取り出す。
           idx = detection[1]
           class_name = classNames[idx]
 
           # 検出された物体の名前を表示
           # print(" "+str(idx) + " " + str(confidence) + " " + class_name)
 
           # 予測値に元の画像サイズを掛けて、四角で囲むための4点の座標情報を得る
           axis = detection[3:7] * (image_width, image_height, image_width, image_height)
 
           # floatからintに変換して、変数に取り出す。画像に四角や文字列を書き込むには、座標情報はintで渡す必要がある。
           (start_X, start_Y, end_X, end_Y) = axis.astype(np.int)[:4]
 
           # (画像、開始座標、終了座標、色、線の太さ)を指定
           cv2.rectangle(frame, (start_X, start_Y), (end_X, end_Y), (23, 230, 210), thickness=2)
 
           # (画像、文字列、開始座標、フォント、文字サイズ、色)を指定
           cv2.putText(frame, class_name, (start_X, start_Y), cv2.FONT_ITALIC, (.005*image_width), (0, 0, 255))       
    
    if ret:
        cv2.imshow(window_name, frame)
        if cv2.waitKey(delay) & 0xFF == ord('q'): # qが押されると終了
            break
    else:
        video.set(cv2.CAP_PROP_POS_FRAMES, 0)

cv2.destroyWindow(window_name)
video.release()