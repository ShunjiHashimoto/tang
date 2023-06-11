# 収穫サポートロボット「TANG」
[CuGo V3](https://cuborex.com/cugo)を使った収穫サポートロボットTANG  
開発背景などの詳細はロボゼミでの[発表スライド](https://docs.google.com/presentation/d/1ee6lWTKakHZedKpi5SkKf4kFQXAP9l1Z/edit#slide=id.p1)をご確認ください 

<img width="400" src="doc/images/tang.png">

## 使用した主な部品
- 足回り
  - [CuGo V3](https://cuborex.com/cugo)
- カメラ
  - RealSense D435i
- 制御用PC
  - Jetson Xavier（Ubuntu 18.04, ROS）
- 画像処理用PC
  - Raspberry Pi 4（Ubuntu 18.04, ROS）
- IMU
  - [WT901C-RS232](https://github.com/witmotion/WT901C-RS232)
- モータドライバ
  - [Cytron SmartDriveDuo 10A](https://www.cytron.io/p-10amp-5v-30v-dc-motor-driver-2-channels)
- 電源
  - モータ駆動用電源 24V
    - LONG 鉛蓄電池 12V 5Ah x2
  - 制御用, 画像処理PC用電源
    - [Anker PowerHouse 90 ポータブル電源](https://www.amazon.co.jp/dp/B0B4KH86H3)

### 画像処理用PC - Jetson Xavier
TODO

## 実行方法
TODO

## ライセンス
[Apache License, Version 2.0](https://www.apache.org/licenses/LICENSE-2.0)
