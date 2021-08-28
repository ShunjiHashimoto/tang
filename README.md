# 収穫サポートロボット「TANG」
CuGo V3を使って収穫サポートロボットTANGを作る

## 必要な開発環境
- Rapberry Pi 4
- Ubuntu18.04が入ったPC
- Webカメラ
- CuGo V3 など

## 必要なパッケージ
### camera
sudo apt-get install ros-melodic-uvc-camera  
sudo apt-get install ros-melodic-image-*  
### joystick
sudo apt-get install ros-melodic-joy
### Raspberry Pi 4
sudo apt-get install python-rpi.gpio

## 実行
ssh ubuntu@raspi.local  
パスワード入力  
source ~/.bashrc  
roslaunch tang_bringup tang_bringup.launch  