#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# 赤色の物体を検出し、追従する

import sys
import numpy as np
from numpy.lib.function_base import copy
import time
import cv2
# ros
import rospy
from tang_msgs.msg import HumanInfo, Modechange, IsDismiss

window_name = 'red detection'
min_area = 300

class DetectRed():
    def __init__(self):
        rospy.init_node('red_detection', anonymous=True)
        # publisher
        self.cmd_publisher = rospy.Publisher('tang_cmd', HumanInfo, queue_size=1)
        self.is_dismiss_publisher = rospy.Publisher('is_dismiss', IsDismiss, queue_size=1)
        self.is_dismiss = IsDismiss()
        # command type
        self.human_info = HumanInfo()
        # camera
        self.video = cv2.VideoCapture(0)
        if not self.video.isOpened(): sys.exit()
        self.debug = rospy.get_param("/tang_detection/debug")
        self.all_time = 0
        self.count = 0

    def calc_mask(self, hsv):
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
    def analysis_blob(self, binary_img):
        # 2値画像のラベリング処理
        # labelは画像のラベリング結果を保持している二次元配列
        label = cv2.connectedComponentsWithStats(binary_img)
        # ブロブ情報を項目別に抽出,背景は0としてラベリングされる
        # label[2]の0行目を削除する、0行目は背景のラベルが格納されている
        data = np.delete(label[2], 0, 0) 
        center = np.delete(label[3], 0, 0)
        # ブロブ面積最大のインデックス
        # 4列目のすべての中から最大値のindexを取得
        try:
            max_index = np.argmax(data[:, 4]) 
        except ValueError:
            # print("ValueError")
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

    def detect_red(self):
        r = rospy.Rate(10)
        human_info = HumanInfo()
        while not rospy.is_shutdown():
            # カメラの画像を１フレーム読み込み、frameに格納、retは読み込めたらtrueを格納する
            ret, frame = self.video.read()
            image_height, image_width = frame.shape[:2]
            frame = cv2.resize(frame , (int(image_width*0.5), int(image_height*0.5)))
            if(not ret):  break
            red_img = frame.copy()
            start = time.time()

            hsv = cv2.cvtColor(red_img, cv2.COLOR_BGR2HSV)
            mask = self.calc_mask(hsv)
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
            target = self.analysis_blob(mask)
            if(target == {}): continue
            if(target["area"] < min_area): continue

                # 面積最大ブロブの中心座標を取得
            center_x = int(target["center"][0])
            center_y = int(target["center"][1])
            radius   = int((target["width"] + target["height"])/4)
            human_info.human_point.x = center_x
            human_info.human_point.y = center_y
            human_info.human_point.z = 0
            human_info.max_area = radius
            self.cmd_publisher.publish(human_info)
            # フレームに面積最大ブロブの中心周囲を円で描く
            cv2.circle(red_img, (center_x, center_y), radius, (0, 200, 0),thickness=2, lineType=cv2.LINE_AA)
            cv2.circle(red_img, (center_x, center_y), 1, (255, 0, 0),thickness=2, lineType=cv2.LINE_AA)
            
            elapsed_time = time.time() - start
            self.count += 1
            self.all_time += elapsed_time
            # print(1/(self.all_time/self.count), "fps")
            # 動画表示
            if ret:
                if(self.debug):
                    cv2.imshow(window_name, red_img)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            else:
                self.video.set(cv2.CAP_PROP_POS_FRAMES, 0)
                print("cant show")

if __name__ == "__main__":
    detect_red = DetectRed()
    detect_red.detect_red()
    rospy.spin()
    detect_red.video.release()
    cv2.destroyWindow(window_name)