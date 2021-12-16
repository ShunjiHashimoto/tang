#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import numpy as np
import matplotlib.pyplot as plt

#認識範囲
xmin,xmax = 40,1060
ymin,ymax = 100,500

def yellow_detect(hsv):
    # HSV色空間に変換
    hsv = cv2.cvtColor(hsv, cv2.COLOR_BGR2HSV)

    # 黄色のHSVの値域
    hsv_min = np.array([20,80,10])
    hsv_max = np.array([30,255,255])
    mask1 = cv2.inRange(hsv, hsv_min, hsv_max)

    # 黄色領域のマスク（255：黄色、0：黄色以外）    
    return(mask1)

# ブロブ解析
def analysis_blob(binary_img):
    # 2値画像のラベリング処理
    label = cv2.connectedComponentsWithStats(binary_img)

    # ブロブ情報を項目別に抽出
    n = label[0] - 1
    data = np.delete(label[2], 0, 0)
    center = np.delete(label[3], 0, 0)

    # ブロブ面積最大のインデックス
    max_index = np.argmax(data[:, 4])

    # 面積最大ブロブの情報格納用
    maxblob = {}

    # 面積最大ブロブの各種情報を取得
    maxblob["upper_left"] = (data[:, 0][max_index], data[:, 1][max_index]) # 左上座標
    maxblob["width"] = data[:, 2][max_index]  # 幅
    maxblob["height"] = data[:, 3][max_index]  # 高さ
    maxblob["area"] = data[:, 4][max_index]   # 面積
    maxblob["center"] = center[max_index]  # 中心座標
    
    return maxblob

def main():
    # フレームを取得
    prev_frame = cv2.imread("/home/hashimoto/catkin_ws/src/tang/tang_detection/figs/dashimaki.png")
    # 領域指定
    frame = prev_frame[ymin:ymax,xmin:xmax] 
    # cv2.rectangle(frame,(xmin,ymin),(xmax,ymax),(0,0,255),2)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 黄色検出
    mask = yellow_detect(hsv)

    # マスク画像をブロブ解析（面積最大のブロブ情報を取得）
    target = analysis_blob(mask)

    # 面積最大ブロブの中心座標を取得
    area = int(target["area"])

    contours, hierarchy = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

    # 小さい輪郭は誤検出として削除する
    contours = list(filter(lambda x: cv2.contourArea(x) > 100, contours))

    # 輪郭を描画
    cv2.drawContours(frame, contours, -1, color=(0, 0, 255), thickness=2)

    cv2.imwrite("/home/hashimoto/catkin_ws/src/tang/tang_detection/figs/dashimaki_masked.png", frame)
    print("success to show yellow mask")

    try:
        plt.imshow(frame)
        print("max area", area)
        plt.show()
    finally:
        plt.close()

if __name__ == '__main__':
    main() 