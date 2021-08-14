#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys # sysはPythonのインタプリタや実行環境に関する情報を扱うためのライブラリです。
import numpy as np
import cv2
import rospy

file_path = '/home/hashimoto/Videos/capture/hue_0_30.mp4'
delay = 1
window_name = 'frame'

class DetectRed():
    def __init__(self):
        rospy.init_node('red_detection', anonymous=True)
        self.video = cv2.VideoCapture(0)
        if not self.video.isOpened():
            sys.exit()
        self.img = cv2.IMREAD_COLOR
        self.count = 0

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


    def maxContours(self, contours):
        max_area = 0
        max_area_contour = -1
        if(len(contours)>0):
            for i in range(0, len(contours)):
                area = cv2.contourArea(contours[i])
                if(max_area < area):
                    max_area = area
                    max_area_contour = i
        return(max_area_contour)

    # ブロブ解析
    def analysis_blob(self, binary_img):
        # 2値画像のラベリング処理
        # labelは画像のラベリング結果を保持している二次元配列
        label = cv2.connectedComponentsWithStats(binary_img)
        if(self.count == 0): 
            print(label[2])

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
            frame = cv2.resize(frame, (int(frame.shape[1]/3), int(frame.shape[0]/3))) # 1/4にリサイズ, shape[1]が高さ, shape[0]が幅
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = self.maskCalc(hsv)
            masked_img = cv2.bitwise_and(frame, frame, mask=mask)

            # マスク画像をブロブ解析（面積最大のブロブ情報を取得）
            target = self.analysis_blob(mask)

             # 面積最大ブロブの中心座標を取得
            center_x = int(target["center"][0])
            center_y = int(target["center"][1])

            # フレームに面積最大ブロブの中心周囲を円で描く
            cv2.circle(frame, (center_x, center_y), 30, (0, 200, 0),thickness=3, lineType=cv2.LINE_AA)

            # マスキング処理
            # masked_img = cv2.bitwise_and(frame, frame, mask=mask)
            # gray_img = cv2.cvtColor(masked_img, cv2.COLOR_BGR2GRAY)
            # ret, threshold_img = cv2.threshold(gray_img, 70, 255, cv2.THRESH_OTSU) # 2値化
            # contours, hierarchy = cv2.findContours(threshold_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            # max_area_contour = self.maxContours(contours)
            # print("no contour")
            # if(max_area_contour == -1): continue
            # print("ok contour")
            # (x,y),radius = cv2.minEnclosingCircle(contours[max_area_contour])
            # center = (int(x),int(y))
            # radius = int(radius)
            # cv2.circle(masked_img,center,radius,(255,0,0),2)

            if ret:
                cv2.imshow(window_name, frame)
                cv2.imshow("original", masked_img)
                print("show image")
                if cv2.waitKey(delay) & 0xFF == ord('q'):
                    break
            else:
                self.video.set(cv2.CAP_PROP_POS_FRAMES, 0)
                print("cant show")
            # r.sleep()


if __name__ == "__main__":
    video = DetectRed()
    video.detectRed()
    rospy.spin()
    video.video.release()
    cv2.destroyWindow(window_name)
