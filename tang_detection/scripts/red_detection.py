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
import roslib.packages
import time

delay = 1
window_name = 'red detection'
pkg_name = 'tang_detection'
min_area = 300
classNames = {0: 'background', 1: 'person'}

class PubMsg():
    # TODO: HumanInfo型でPublishする
    def __init__(self):
        self.publisher = rospy.Publisher('msg_topic', String, queue_size=10)

    def pub(self, center_x, radius, mode):
        # print(center_x, radius)
        if(mode == 'human'): 
            radius_max = 50
        else:
            radius_max = 70

        if(0 <= center_x and center_x <= 80 and radius < radius_max):
            str = "turn left"
        elif(240 < center_x and center_x <= 320 and radius < radius_max):
            str = "turn right"
        else:
            if(radius >= radius_max or radius <= 10):
                str = "stop"
            else:
                str = "go ahead"
        # rospy.loginfo(str)
        self.publisher.publish(str)

class DetectRed():
    def __init__(self):
        rospy.init_node('red_detection', anonymous=True)
        self.video = cv2.VideoCapture(0)
        self.debug = rospy.get_param("/tang_detection/debug")
        self.joy_sub = rospy.Subscriber("joy", Joy, self.joyCallback, queue_size=1)
        self.mode = 'red'
        self.center_x = 0
        self.radius = 0
        self.center_y = 0
        self.all_time = 0
        self.count = 0
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
        try:
            max_index = np.argmax(data[:, 4]) # 4列目のすべての中から最大値のindexを取得
        except ValueError:
            print("ValueError")
            maxblob = {}
            return maxblob

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
            image_height, image_width = frame.shape[:2]
            frame = cv2.resize(frame , (int(image_width*0.5), int(image_height*0.5)))
            if(not ret):  break
            red_img = frame.copy()
            start = time.time()

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
                if(target == {}): continue
                if(target["area"] < min_area): continue

                 # 面積最大ブロブの中心座標を取得
                self.center_x = int(target["center"][0])
                self.center_y = int(target["center"][1])
                self.radius   = int((target["width"] + target["height"])/4)
                pubmsg = PubMsg()
                pubmsg.pub(self.center_x, self.radius, self.mode)
                # フレームに面積最大ブロブの中心周囲を円で描く
                cv2.circle(red_img, (self.center_x, self.center_y), self.radius, (0, 200, 0),thickness=2, lineType=cv2.LINE_AA)
                cv2.circle(red_img, (self.center_x, self.center_y), 1, (255, 0, 0),thickness=2, lineType=cv2.LINE_AA)
            
            elapsed_time = time.time() - start
            self.count += 1
            self.all_time += elapsed_time
            print(1/(self.all_time/self.count), "fps")
            # 動画表示
            if ret:
                if(self.debug):
                    cv2.imshow(window_name, red_img)
                if cv2.waitKey(delay) & 0xFF == ord('q'):
                    break
            else:
                self.video.set(cv2.CAP_PROP_POS_FRAMES, 0)
                print("cant show")

if __name__ == "__main__":
    detect_red = DetectRed()
    detect_red.detectRed()
    rospy.spin()
    detect_red.video.release()
    cv2.destroyWindow(window_name)