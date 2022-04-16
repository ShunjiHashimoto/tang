#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 赤色の物体を検出後、位置を特定する
import sys 
import numpy as np
import cv2
import rospy
from std_msgs.msg import String
import roslib.packages
from tang_msgs.msg import Command


delay = 1
window_name = 'red detection'
pkg_name = 'tang_detection'
min_area = 300

class DetectRed():
    """
    @class DetectRed
    @brief 赤検出クラス
    """
    def __init__(self):
        pass

    def mask_calc(self, hsv):
        """
        @fn mask_calc()
        @param hsv hsv値で表現された画像
        @details 赤色とそれ以外で二値化
        """
        # 赤色のHSVの値域1(hはもとは9~30らしい)
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
        """
        @fn analysis_blob()
        @param binary_img 二値化された画像
        @details 最も大きい面積の位置情報と大きさを取得
        """
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

    def red_detection(self, frame, ret):
        """
        @fn red_detection(self, frame, ret)
        @param frame, ret 背景処理された画像
        @return radius, pos_x
        @details 赤色検出
        """
        r = rospy.Rate(10) 
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = self.mask_calc(hsv)
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
        if(target["area"] < min_area): return

         # 面積最大ブロブの中心座標を取得
        center_x = int(target["center"][0])
        center_y = int(target["center"][1])
        radius = int((target["width"] + target["height"])/4)

        # フレームに面積最大ブロブの中心周囲を円で描く
        # cv2.circle(frame, (center_x, center_y), radius, (0, 200, 0),thickness=2, lineType=cv2.LINE_AA)
        # cv2.circle(frame, (center_x, center_y), 1, (255, 0, 0),thickness=2, lineType=cv2.LINE_AA)

        if ret:
            cv2.imshow(window_name, frame)
            cv2.imshow("masked_img", h)
            # if cv2.waitKey(delay) & 0xFF == ord('q'):
            #     return
            return 
        else:
            self.video.set(cv2.CAP_PROP_POS_FRAMES, 0)
            print("cant show")
            # r.sleep()
