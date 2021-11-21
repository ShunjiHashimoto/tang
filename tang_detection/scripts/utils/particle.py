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
min_area = 300

class ParticleFilter:
    @classmethod
    # 尤度計算、現在のパーティクルの位置が赤色であるかどうかの確率計算
    # 引数：パーティクルの位置、画像
    # その位置の周りの画素数、その位置のhsv
    # 返り値：そのパーティクルの尤度w
    def isTarget(cls, roi):
        return (roi<=6) | (roi >=150)

    def calcLikelihood(cls, x, y, img, w=50, h=50):
        x1, y1 = max(0, x-w/2), max(0, y-h/2) # x,y座標ではマイナス値がないため、maxで最低0になるようにしてる
        x2, y2 = min(img.shape[1], x+w/2), min(img.shape[0], y+h/2) # shape[0]:行数 shape[1]：列数
        x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
        roi = img[y1:y2, x1:x2] # region of interest, 注目領域
        # 注目領域においてどれだけ赤があるか
        count = roi[cls.isTarget(roi)].size
        return (float(count) / img.size) if count > 0 else 0.0001
        
    def initialize(cls, img, x, y, N):
        ps = np.ndarray((N, 3), dtype=np.float32)  # パーティクル格納用の配列を生成
        # 尤度計算
        w = cls.calcLikelihood(x, y, img)
        ps[:] = [x, y, w]
        return ps
    
    def doResampling(cls, ps):
        # 累積重みの計算
        ws = ps[:, 2].cumsum() # 重みの合計値を計算(1, 1, 1, 1, 1) -> (1, 2, 3, 4, /・・)
        last_w = ws[ws.shape[0] - 1] # 最後の重み合計値
        # 新しいパーティクル用の空配列を生成
        new_ps = np.empty(ps.shape) # shapeで配列の要素数を返す
        # 前状態の重みに応じてパーティクルをリサンプリング（重みは1.0）
        for i in range(ps.shape[0]):
            w = np.random.rand() * last_w # 前回の重みの合計値 * (0~1)
            new_ps[i] = ps[(ws > w).argmax()] # ws配列をスキャンして、初めてwよりも大きな値がの配列インデックスを返す
            new_ps[i, 2] = 1.0 # 重みを1にセットする
        return new_ps
    
    def positionPredict(cls, ps, var=10.0):
        # 分散に従ってランダムに少し位置をずらす
        ps[:, 0] += np.random.randn((ps.shape[0])) * var # 0列目の値、つまりz座標の配列にランダムに値を加える
        ps[:, 1] += np.random.randn((ps.shape[0])) * var
        return ps

    def calcWeight(cls, ps, img):
        # 尤度に従ってパーティクルの重み付け
        for i in range(ps.shape[0]):
            ps[i][2] = cls.calcLikelihood(ps[i, 0], ps[i, 1], img)

        # 重みの正規化
        ps[:, 2] *= ps.shape[0] / ps[:, 2].sum() # パーティクルの数/パーティクルの重みの合計値
        return ps
    
    # 成績の良いパーティクルが集中している付近の位置を割り出す
    def observer(cls, ps, img):
        # パーティクルの重み付け
        ps = cls.calcWeight(ps, img)
        # 重み和の計算
        x = (ps[:, 0] * ps[:, 2]).sum() # (パーティクルのx座標 * 重み)の合計値
        y = (ps[:, 1] * ps[:, 2]).sum() # (パーティクルのy座標 * 重み)の合計値
        # 重み付き平均を返す
        return (x, y) / ps[:, 2].sum()


    def particleFilter(cls, ps, img, center_x, center_y, N=500):
        if(ps is None):
                ps = ParticleFilter().initialize(img, center_x, center_y, 10)
        ps = cls.doResampling(ps)
        ps = cls.positionPredict(ps)
        x, y = cls.observer(ps, img)
        return ps, int(x), int(y)

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
        ps = None
        while not rospy.is_shutdown():
            ret, frame = self.video.read() # カメラの画像を１フレーム読み込み、frameに格納、retは読み込めたらtrueを格納する
            if(not ret): 
                print("error")
                continue
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
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
            if(target["area"] < min_area): continue

             # 面積最大ブロブの中心座標を取得
            center_x = int(target["center"][0])
            center_y = int(target["center"][1])
            radius = int((target["width"] + target["height"])/4)

            # フレームに面積最大ブロブの中心周囲を円で描く
            cv2.circle(frame, (center_x, center_y), radius, (0, 200, 0),thickness=2, lineType=cv2.LINE_AA)
            cv2.circle(frame, (center_x, center_y), 1, (255, 0, 0),thickness=2, lineType=cv2.LINE_AA)

            # particle_fileterを使って、赤色の中心位置を推定する
            # その結果を画像に表示する、フィルタをかける前と書けた後を比較する
            # 1.particlesに(x, y, w)を格納する、なければ初期化を行う
            ps, x, y = ParticleFilter().particleFilter(ps, h, center_x, center_y, 20)

            # 画像の範囲内にあるパーティクルのみ取り出し
            ps1 = ps[(ps[:, 0] >= 0) & (ps[:, 0] < frame.shape[1]) &
                     (ps[:, 1] >= 0) & (ps[:, 1] < frame.shape[0])]

            for i in range(ps1.shape[0]):
                frame[int(ps1[i, 1]), int(ps1[i, 0])] = [0, 200, 0]
            cv2.rectangle(frame, (x-20, y-20), (x+20, y+20), (0, 0, 200), 5)

            if ret:
                cv2.imshow(window_name, frame)
                cv2.imshow("masked_img", h)
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
