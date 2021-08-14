#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys # sysはPythonのインタプリタや実行環境に関する情報を扱うためのライブラリです。
import numpy as np
import cv2
import rospy
import math

file_path = '/home/hashimoto/Videos/capture/hue_0_30.mp4'
delay = 1
window_name = 'red detection'
min_area = 1000

class Particle():
    def __init__(self, init_pose):
        self.pose = init_pose
        self.updated_pose = np.array([0, 0]).T

    # particleの更新、ランダムにparticleを動かす
    def motion_update(self):
        self.updated_pose[0] = self.pose[0] + np.random.rand()
        self.updated_pose[1] = self.pose[1] + np.random.rand()

# MonteCarloLocalization
class Mcl():
    def __init__(self, init_pose, num):
        self.particles = [Particle(init_pose) for i in range(num)]
    
    def motion_update(self):
        for p in self.particles:
            p.motion_update()

class DetectRed():
    def __init__(self):
        rospy.init_node('red_detection', anonymous=True)
        self.video = cv2.VideoCapture(0)
        if not self.video.isOpened():
            sys.exit()
        self.img = cv2.IMREAD_COLOR

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
        while not rospy.is_shutdown():
            ret, frame = self.video.read() # カメラの画像を１フレーム読み込み、frameに格納、retは読み込めたらtrueを格納する
            if(not ret): 
                print("error")
                continue
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = self.maskCalc(hsv)
            masked_img = cv2.bitwise_and(frame, frame, mask=mask)

            # マスク画像をブロブ解析（面積最大のブロブ情報を取得）
            target = self.analysisBlob(mask)
            if(target["area"] < min_area): continue

             # 面積最大ブロブの中心座標を取得
            center_x = int(target["center"][0])
            center_y = int(target["center"][1])
            radius = int((target["width"] + target["height"])/4)

            # フレームに面積最大ブロブの中心周囲を円で描く
            cv2.circle(frame, (center_x, center_y), radius, (0, 200, 0),thickness=2, lineType=cv2.LINE_AA)
            cv2.circle(frame, (center_x, center_y), 1, (255, 0, 0),thickness=2, lineType=cv2.LINE_AA)
            initial_pose = np.array([center_x, center_y]).T
            mcl = Mcl(initial_pose, 5)
            mcl.motion_update() # Mcl.motion_update() ５個のパーティクルに対して処理を行う
            for p in mcl.particles:
                cv2.circle(frame, (p.updated_pose[0], p.updated_pose[1]), 1, (255, 0, 0),thickness=2, lineType=cv2.LINE_AA)

            if ret:
                cv2.imshow(window_name, frame)
                cv2.imshow("masked_img", masked_img)
                if cv2.waitKey(delay) & 0xFF == ord('q'):
                    break
            else:
                self.video.set(cv2.CAP_PROP_POS_FRAMES, 0)
                print("cant show")
            # r.sleep()

if __name__ == "__main__":
    detect_red = DetectRed()
    detect_red.detectRed()
    rospy.spin()
    detect_red.video.release()
    cv2.destroyWindow(window_name)
