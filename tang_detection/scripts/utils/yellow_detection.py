#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import numpy as np
import matplotlib.pyplot as plt

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
    imgfile_path = "/home/hashimoto/git/tang/tang_detection/figs/dashimaki_left.png"

    # カメラのキャプチャ
    frame = cv2.imread(imgfile_path)
    # BGR->RGB
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # 黄色のHSVの値域
    hsv_min = np.array([54, 100 ,89])
    hsv_max = np.array([115, 255 ,255])
    # 黄色でmaskをかける
    mask = cv2.inRange(hsv, hsv_min, hsv_max)

    # find tulips
    labels, contours, hierarchy = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

    for i in range(0, len(contours)):
        if len(contours[i]) > 0:
            # remove small objects
            if cv2.contourArea(contours[i]) < 500:
                continue
            cv2.polylines(frame, contours[i], True, (0, 255, 0), 5)

    # 画像を縦に2分割
    height, width, channels = frame.shape
    frame = frame[0:height, 0:width/2]
    plt.imshow(frame)
    plt.show()
    print("success to show yellow mask")

if __name__ == '__main__':
    main() 