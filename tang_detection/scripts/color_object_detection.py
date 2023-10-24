#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# 若草色の物体を検出し、追従する

import sys
import numpy as np
from numpy.lib.function_base import copy
import time
import cv2
# ros
import rospy
from tang_msgs.msg import HumanInfo, IsDismiss
# param
from config import ColorObjectParam, CameraParam, HumanFollowParam

class DetectColorObject():
    def __init__(self):
        rospy.init_node('color_object_detection', anonymous=True)
        # publisher
        self.cmd_publisher = rospy.Publisher('tang_cmd', HumanInfo, queue_size=1)
        self.is_dismiss_publisher = rospy.Publisher('is_dismiss', IsDismiss, queue_size=1)
        # command type
        self.human_info = HumanInfo()
        self.is_dismiss = IsDismiss()
        # camera
        self.video = cv2.VideoCapture(0)
        self.video.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
        self.video.set(cv2.CAP_PROP_FPS, CameraParam.fps)
        self.video.set(cv2.CAP_PROP_FRAME_WIDTH, CameraParam.width) 
        self.video.set(cv2.CAP_PROP_FRAME_HEIGHT, CameraParam.height)
        fourcc = self.decode_fourcc(self.video.get(cv2.CAP_PROP_FOURCC))
        width = self.video.get(cv2.CAP_PROP_FRAME_WIDTH)
        height = self.video.get(cv2.CAP_PROP_FRAME_HEIGHT)
        fps = self.video.get(cv2.CAP_PROP_FPS)
        print("fourcc:{} fps:{} width:{} height:{}".format(fourcc, fps, width, height))
        if not self.video.isOpened(): sys.exit()
        # process time
        self.prev_time = 0.0
        # rosparam
        self.debug = rospy.get_param("/tang_detection/debug")
    
    def decode_fourcc(self, v):
        v = int(v)
        return "".join([chr((v >> 8 * i) & 0xFF) for i in range(4)])
    
    def calc_delta_time(self):
        now = rospy.Time.now().to_sec()
        delta_t = now - self.prev_time
        self.prev_time = now
        return delta_t

    def calc_mask(self, hsv):
        # 緑色のHSVの値域
        hsv_min = np.array([ColorObjectParam.hue_min, ColorObjectParam.sat_min, ColorObjectParam.val_min]) 
        hsv_max = np.array([ColorObjectParam.hue_max, ColorObjectParam.sat_max, ColorObjectParam.val_max])
        # 緑色領域のマスク
        mask = cv2.inRange(hsv, hsv_min, hsv_max)
        return(mask)
        
    # ブロブ解析
    def analysis_blob(self, binary_img):
        # 2値画像のラベリング処理
        # labelは画像のラベリング結果を保持している二次元配列
        label = cv2.connectedComponentsWithStats(binary_img)
        # ブロブ情報を項目別に抽出,背景は0としてラベリングされる
        # label[2]の0行目を削除する、0行目は背景のラベルが格納されている
        data = np.delete(label[2], 0, 0) 
        center = np.delete(label[3], 0, 0)
        # 面積最大ブロブの情報格納用
        maxblob = {}
        # ブロブ面積最大のインデックス
        # 4列目のすべての中から最大値のindexを取得
        try:
            max_index = np.argmax(data[:, 4]) 
        except ValueError:
            maxblob["area"] = 0
            maxblob["center"] = [0, 0]
            maxblob["width"] = 0
            maxblob["height"] = 0
            return maxblob
        # 面積最大ブロブの各種情報を取得
        maxblob["upper_left"] = (data[:, 0][max_index], data[:, 1][max_index]) # 左上座標(x, y)
        maxblob["width"] = data[:, 2][max_index]  # 幅
        maxblob["height"] = data[:, 3][max_index]  # 高さ
        maxblob["area"] = data[:, 4][max_index]   # 面積
        maxblob["center"] = center[max_index]  # 中心座標
        return maxblob
    
    def calc_human_info(self, maxblob):
        center_y = int(maxblob["center"][0]) # 画像の横方向
        center_z = int(maxblob["center"][1]) # 画像の上下方向
        radius   = int((maxblob["width"] + maxblob["height"])/4)
        self.human_info.human_point.x = 0
        self.human_info.human_point.y = center_y
        self.human_info.human_point.z = center_z
        self.human_info.max_area = radius
        return

    def detect_color_object(self):
        dismiss_human_time = 0.0
        delta_t = self.calc_delta_time()
        ret, frame = self.video.read()
        while not rospy.is_shutdown():
            self.is_dismiss.flag = False
            delta_t = self.calc_delta_time()
            # カメラの画像を１フレーム読み込み、frameに格納、retは読み込めたらtrueを格納する
            ret, frame = self.video.read()
            image_height, image_width = frame.shape[:2]
            frame = cv2.resize(frame , (int(image_width*1.0), int(image_height*1.0)))
            if(not ret):  break
            target_img = frame.copy()

            hsv = cv2.cvtColor(target_img, cv2.COLOR_BGR2HSV)
            mask = self.calc_mask(hsv)
            # HSV画像からH(色相)成分を抽出
            h = hsv[:, :, 0]

            # S(彩度)成分を二値化（大津の手法を利用して最適な閾値で二値化）
            # この操作で彩度の低い部分と高い部分が分けられる
            ret, s = cv2.threshold(hsv[:, :, 1], 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)

            # V(明度)成分も同様に二値化
            # この操作で明るい部分と暗い部分が分けられる
            ret, v = cv2.threshold(hsv[:, :, 2], 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)

            # SまたはVの二値化画像で0(黒)と判定された部分は、色の情報が不足していると判断
            # これは、彩度が低すぎる、または明度が低すぎるため、その部分の色相Hを100にして識別しやすくする
            # 100という値は、対象とは異なる色を示す目印として使用される（任意の値で良い）
            h[(s == 0) | (v == 0)] = 100

            # マスク画像をブロブ解析（面積最大のブロブ情報を取得）
            maxblob = self.analysis_blob(mask)
            # 人を見失った場合は停止する
            if(maxblob["area"] < ColorObjectParam.object_min_area):
                dismiss_human_time += delta_t
                if dismiss_human_time > HumanFollowParam.dismiss_time:
                    self.human_info.is_human = 0
                    # rospy.loginfo("Dismiss human : %lf", dismiss_human_time)
                    self.is_dismiss.flag = True
                    self.is_dismiss_publisher.publish(self.is_dismiss)
                    continue
            else:
                self.is_dismiss.flag  = False
                dismiss_human_time = 0.0
                self.is_dismiss_publisher.publish(self.is_dismiss)
            # 面積最大ブロブの中心座標を取得
            self.calc_human_info(maxblob)
            self.cmd_publisher.publish(self.human_info)
            # フレームに面積最大ブロブの中心周囲を円で描く
            cv2.circle(target_img, (self.human_info.human_point.y, self.human_info.human_point.z), self.human_info.max_area, (0, 200, 0),thickness=2, lineType=cv2.LINE_AA)
            cv2.circle(target_img, (self.human_info.human_point.y, self.human_info.human_point.z), 1, (255, 0, 0),thickness=2, lineType=cv2.LINE_AA)
            print(f"target object max area: {self.human_info.max_area}")
            # 動画表示
            if ret:
                if(self.debug):
                    cv2.imshow(CameraParam.window_name, target_img)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            else:
                self.video.set(cv2.CAP_PROP_POS_FRAMES, 0)
                print("cant show")

if __name__ == "__main__":
    detect_color_object = DetectColorObject()
    detect_color_object.detect_color_object()
    rospy.spin()
    detect_color_object.video.release()
    cv2.destroyWindow(CameraParam.window_name)