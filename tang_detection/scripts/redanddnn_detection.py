#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# 赤色の物体を検出かつ人を検知すれば追従

import sys # sysはPythonのインタプリタや実行環境に関する情報を扱うためのライブラリです。
import numpy as np
from numpy.lib.function_base import copy
import cv2
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Joy

delay = 1
window_name = 'red detection'
min_area = 300

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

model = cv2.dnn.readNetFromTensorflow('/home/hashimoto/catkin_ws/src/tang/tang_detection/models/frozen_inference_graph.pb',
                                      '/home/hashimoto/catkin_ws/src/tang/tang_detection/models/ssd_mobilenet_v2_coco_2018_03_29.pbtxt')
                                


class PubMsg():
    def __init__(self):
        self.publisher = rospy.Publisher('msg_topic', String, queue_size=10)

    def pub(self, center_x, radius):
        if(0 <= center_x and center_x <= 240 and radius < 130):
            str = "turn left"
        elif(400 < center_x and center_x <= 640 and radius < 130):
            str = "turn right"
        else:
            if(radius >= 130 or radius <= 10):
                str = "stop"
            else:
                str = "go ahead"
        rospy.loginfo(str)
        self.publisher.publish(str)

class OpencvDnn():
    @classmethod
    def humanEstimation(cls, frame, center_x, center_y, radius):
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
                
                # 人以外はスキップ
                if(class_name is not "person"): continue
 
                # 予測値に元の画像サイズを掛けて、四角で囲むための4点の座標情報を得る
                axis = detection[3:7] * (image_width, image_height, image_width, image_height)
 
                # floatからintに変換して、変数に取り出す。画像に四角や文字列を書き込むには、座標情報はintで渡す必要がある。
                (start_X, start_Y, end_X, end_Y) = axis.astype(np.int)[:4]

                # 赤色の中心座標が人物領域に入ってたらその赤色の中心座標を返し、その座標をPub
                pubmsg = PubMsg()
                if(start_X <= center_x and center_x <= end_X and start_Y <= center_y and center_y <= end_Y):
                    pubmsg.pub(center_x, radius)
                
                if(center_x == 0 and center_y == 0):
                    center_x = (start_X + end_X)/2
                    center_y = (start_Y + end_Y)/2
                    radius = abs(start_X - end_X)/3
                    pubmsg.pub(center_x, radius)
                    cv2.circle(frame, (int(center_x), int(center_y)), int(radius), (0, 200, 0),thickness=2, lineType=cv2.LINE_AA)
 
                # (画像、開始座標、終了座標、色、線の太さ)を指定
                cv2.rectangle(frame, (start_X, start_Y), (end_X, end_Y), (23, 230, 210), thickness=2)
 
                # (画像、文字列、開始座標、フォント、文字サイズ、色)を指定
                cv2.putText(frame, class_name, (start_X, start_Y), cv2.FONT_ITALIC, (.005*image_width), (0, 0, 255))             

        return frame

class DetectRed():
    def __init__(self):
        rospy.init_node('red_detection', anonymous=True)
        self.video = cv2.VideoCapture(rospy.get_param("/tang_detection/video_path"))
        self.debug = rospy.get_param("/tang_detection/debug")
        self.joy_sub = rospy.Subscriber("joy", Joy, self.joyCallback, queue_size=1)
        self.mode = None
        self.center_x = 0
        self.radius = 0
        self.center_y = 0
        if not self.video.isOpened():
            sys.exit()
        self.img = cv2.IMREAD_COLOR

    def joyCallback(self, joy_msg):
        if(joy_msg.buttons[1]):
            self.mode = 'red'
        elif(joy_msg.buttons[2]):
            self.mode = 'human'
        elif(joy_msg.buttons[4] or joy_msg.buttons[5]):
            self.mode = 'redhuman'
        elif(joy_msg.buttons[7]):
            self.mode = 'stop'

    def maskCalc(self, hsv):
        # 赤色のHSVの値域1
        hsv_min = np.array([1,128,0]) # 赤色の小さい値を除去
        hsv_max = np.array([6,255,255])
        mask1 = cv2.inRange(hsv, hsv_min, hsv_max)

        # 赤色のHSVの値域2
        hsv_min = np.array([150,200,0])
        hsv_max = np.array([179,255,255])
        mask2 = cv2.inRange(hsv, hsv_min, hsv_max)

        # 赤色領域のマスク（255：赤色、0：赤色以外）    
        return(mask1 + mask2)

    # ブロブ解析
    def analysisBlob(self, binary_img):
        # 2値画像のラベリング処理
        # labelは画像のラベリング結果を保持している二次元配列
        label = cv2.connectedComponentsWithStats(binary_img)

        # ブロブ情報を項目別に抽出,背景は0としてラベリングされる
        data = np.delete(label[2], 0, 0) # label[2]の0行目を削除する、0行目は背景のラベルが格納されている
        center = np.delete(label[3], 0, 0)

        # ブロブ面積最大のインデックス
        max_index = np.argmax(data[:, 4]) # 4列目のすべての中から最大値のindexを取得

        # 面積最大ブロブの情報格納用
        maxblob = {}

        # 面積最大ブロブの各種情報を取得
        maxblob["upper_left"] = (data[:, 0][max_index], data[:, 1][max_index]) # 左上座標(x, y)
        maxblob["width"] = data[:, 2][max_index]  # 幅
        maxblob["height"] = data[:, 3][max_index]  # 高さ
        maxblob["area"] = data[:, 4][max_index]   # 面積
        maxblob["center"] = center[max_index]  # 中心座標
        return maxblob

    def detectRed(self):
        # video = self.readVideo()
        r = rospy.Rate(10)
        # center_x, center_y, radius = 0
        while not rospy.is_shutdown():
            ret, frame = self.video.read() # カメラの画像を１フレーム読み込み、frameに格納、retは読み込めたらtrueを格納する
            if(not ret):  break
            red_img = frame.copy()
            human_img = frame.copy()

            # 赤色検出のみ
            if(self.mode == 'red'):
                hsv = cv2.cvtColor(red_img, cv2.COLOR_BGR2HSV)
                mask = self.maskCalc(hsv)
                # masked_img = cv2.bitwise_and(frame, frame, mask=mask)
                h = hsv[:, :, 0] # ０列目の列をすべて抽出、この場合hだけを抽出
                # S, Vを2値化（大津の手法）
                ret, s = cv2.threshold(hsv[:, :, 1], 0, 255,
                                   cv2.THRESH_BINARY | cv2.THRESH_OTSU) # 1列目を抽出(s値)
                ret, v = cv2.threshold(hsv[:, :, 2], 0, 255,
                                   cv2.THRESH_BINARY | cv2.THRESH_OTSU) # 2列目を抽出(v値)
                 # s,vどちらかが0であれば、そのh[]の値を100にする、つまり赤色ではない色
                 # hの配列の中でTrueになった箇所だけを操作する
                h[(s == 0) | (v == 0)] = 100

                # マスク画像をブロブ解析（面積最大のブロブ情報を取得）
                target = self.analysisBlob(mask)
                if(target["area"] < min_area): continue

                 # 面積最大ブロブの中心座標を取得
                self.center_x = int(target["center"][0])
                self.center_y = int(target["center"][1])
                self.radius   = int((target["width"] + target["height"])/4)
                pubmsg = PubMsg()
                pubmsg.pub(self.center_x, self.radius)
                # フレームに面積最大ブロブの中心周囲を円で描く
                cv2.circle(red_img, (self.center_x, self.center_y), self.radius, (0, 200, 0),thickness=2, lineType=cv2.LINE_AA)
                cv2.circle(red_img, (self.center_x, self.center_y), 1, (255, 0, 0),thickness=2, lineType=cv2.LINE_AA)

            # 人物検出開始のみ
            elif(self.mode == 'human'):
                self.center_x = 0
                self.center_y = 0
                opencv_dnn = OpencvDnn()
                human_img = opencv_dnn.humanEstimation(frame, self.center_x, self.center_y, self.radius)
                # cv2.imshow("masked_img", human_img)
                # center_x, center_y, radiusを算出
                # cv2.circle(human_img, (start_x, start_y), 1, (255, 0, 0),thickness=2, lineType=cv2.LINE_AA)
            
            # 赤色+人物検出
            elif(self.mode == 'redhuman'):
                hsv = cv2.cvtColor(red_img, cv2.COLOR_BGR2HSV)
                mask = self.maskCalc(hsv)
                # masked_img = cv2.bitwise_and(frame, frame, mask=mask)
                h = hsv[:, :, 0] # ０列目の列をすべて抽出、この場合hだけを抽出
                # S, Vを2値化（大津の手法）
                ret, s = cv2.threshold(hsv[:, :, 1], 0, 255,
                                   cv2.THRESH_BINARY | cv2.THRESH_OTSU) # 1列目を抽出(s値)
                ret, v = cv2.threshold(hsv[:, :, 2], 0, 255,
                                   cv2.THRESH_BINARY | cv2.THRESH_OTSU) # 2列目を抽出(v値)
                 # s,vどちらかが0であれば、そのh[]の値を100にする、つまり赤色ではない色
                 # hの配列の中でTrueになった箇所だけを操作する
                h[(s == 0) | (v == 0)] = 100

                # マスク画像をブロブ解析（面積最大のブロブ情報を取得）
                target = self.analysisBlob(mask)
                if(target["area"] < min_area): continue

                 # 面積最大ブロブの中心座標を取得
                self.center_x = int(target["center"][0])
                self.center_y = int(target["center"][1])
                self.radius   = int((target["width"] + target["height"])/4)
                # 人物検出開始
                opencv_dnn = OpencvDnn()
                human_img = opencv_dnn.humanEstimation(frame, self.center_x, self.center_y, self.radius)
                cv2.circle(red_img, (self.center_x, self.center_y), self.radius, (0, 200, 0),thickness=2, lineType=cv2.LINE_AA)
                cv2.circle(red_img, (self.center_x, self.center_y), 1, (255, 0, 0),thickness=2, lineType=cv2.LINE_AA)
            
            # 動画表示
            if ret:
                if(self.debug):
                    cv2.imshow(window_name, red_img)
                    cv2.imshow("human_img", human_img)
                if cv2.waitKey(delay) & 0xFF == ord('q'):
                    break
            else:
                self.video.set(cv2.CAP_PROP_POS_FRAMES, 0)
                print("cant show")
            
            # radiusとcenter_xをpub
            # r.sleep()

if __name__ == "__main__":
    detect_red = DetectRed()
    detect_red.detectRed()
    rospy.spin()
    detect_red.video.release()
    cv2.destroyWindow(window_name)
