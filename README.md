# 収穫サポートロボット「TANG」
CuGo V3を使って収穫サポートロボットTanGを作る

## 必要な開発環境
- Rapberry Pi 4
- Ubuntu18.04が入ったPC
- Webカメラ
- CuGo V3 など

## 必要なパッケージ
### pip3
```bash
sudo apt install python3-pip  
```
もしエラーが起きれば以下のサイトを参照する
- 前回電源を切ったときに何かしら不具合が生じ、sudo apt 系ができなくなる現象
https://github.com/ShunjiHashimoto/tang/issues/4#issuecomment-907743709
- ROSの署名が古い
https://github.com/ShunjiHashimoto/tang/issues/4#issuecomment-907744118

### Opencv
```bash
sudo apt update 
sudo apt -y upgrade
pip install --upgrade pip
pip3 install opencv-python
pip3 install opencv-contrib-python
```
参考：https://github.com/ShunjiHashimoto/tang/issues/8

### camera
```bash
sudo apt-get install ros-melodic-uvc-camera  
sudo apt-get install ros-melodic-image-*  
```
### joystick
```bash
sudo apt-get install ros-melodic-joy
```
### Raspberry Pi 4
```bash
sudo apt-get install python-rpi.gpio
```

## 実行
ssh ubuntu@raspi.local  
パスワード入力  
source ~/.bashrc  
roslaunch tang_bringup tang_bringup.launch  
