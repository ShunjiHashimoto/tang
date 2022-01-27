#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import numpy as np
import matplotlib.pyplot as plt
import time

video_path = "/home/hashimoto/git/tang/tang_detection/figs/dashimaki1.MOV"

def main():
    cap = cv2.VideoCapture(0)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3))
    #  履歴の長さ(history)，混合数(nmixtures)，しきい値(backgroundRatio)
    # fgbg = cv2.bgsegm.createBackgroundSubtractorMOG(history=3, nmixtures=5, backgroundRatio=0.1)
    fgbg = cv2.bgsegm.createBackgroundSubtractorGMG()  
    print(type(fgbg))
    mx = prev_mx= 0

    while(1):
        ret, frame = cap.read()
        # 指定した大きさにリサイズする。
        frame = cv2.resize(frame, dsize=(700, 500))
        gusi_frame = cv2.GaussianBlur(frame, (15,15), 3)
        # BGR->RGB
        frame_bgr = cv2.cvtColor(gusi_frame, cv2.COLOR_BGR2RGB)
        # HSVに変換
        hsv_img = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        # 黄色のHSVの値域
        hsv_min = np.array([54, 100 ,89])
        hsv_max = np.array([100, 255 ,255])
        # 黄色でmaskをかける
        mask = cv2.inRange(hsv_img, hsv_min, hsv_max)

        fgmask = fgbg.apply(mask)
        fgmask = cv2.morphologyEx(fgmask, cv2.MORPH_OPEN, kernel)
        # 白の数を数える。
        whitePixels = cv2.countNonZero(fgmask)
        mx = whitePixels if whitePixels > mx else mx
        # 結果の表示, 最大変化量100000
        # もし、背景差分量が大きすぎればピッキングを開始しない。それ以外はピッキングを開始する
        # 見え隠れする場合も動いたとみなされる。。。。と思ったけどその心配はなさそう
        print(mx)

        cv2.imshow('original',frame)
        cv2.imshow('backgroundremove',fgmask)
        cv2.imshow('yellowremove',mask)
        k = cv2.waitKey(30) & 0xff
        if k == 27:
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main() 