#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import numpy as np

def red_detect(hsv):
    # HSV色空間に変換
    hsv = cv2.cvtColor(hsv, cv2.COLOR_BGR2HSV)

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
    imgfile_path = "/home/hashimoto/git/tang/tang_detection/figs/dashimaki.png"

    # カメラのキャプチャ
    cap = cv2.VideoCapture(imgfile_path)
    while(cap.isOpened()):
    
        # フレームを取得
        ret, frame = cap.read()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 赤色検出
        mask = red_detect(hsv)

        # マスク画像をブロブ解析（面積最大のブロブ情報を取得）
        target = analysis_blob(mask)

        # 面積最大ブロブの中心座標を取得
        center_x = int(target["center"][0])
        center_y = int(target["center"][1])

        # フレームに面積最大ブロブの中心周囲を円で描く
        cv2.circle(frame, (center_x, center_y), 30, (0, 200, 0),
                   thickness=3, lineType=cv2.LINE_AA)

        # 結果表示
        cv2.imshow("Frame", frame)
        cv2.imshow("Mask", mask)
        print("success to show yellow mask")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main() 