# 収穫サポートロボット「TANG」
CuGo V3を使って収穫サポートロボットTANGを作る

## 必要な開発環境
- Rapberry Pi 4
- Ubuntu18.04が入ったPC
- Webカメラ
- CuGo V3 など

## 必要なパッケージ
### Ubuntu serverをダウンロードし、SDカードに書き込む
Ubuntuが入ったPCでUbuntu serverをダウンロード
```bash
$ wget http://cdimage.ubuntu.com/releases/bionic/release/ubuntu-18.04.5-preinstalled-server-arm64+raspi3.img.xz
```
ファイルを解凍
```bash
$ xz -dv ubuntu-18.04.5-preinstalled-server-arm64+raspi3.img.xz 
```
Raspberry Pi Imagerを使って書き込む
書き込むimgファイルは先程選択したファイルを選択後、書き込む
![Screenshot from 2021-10-08 07-17-44](https://user-images.githubusercontent.com/63869336/136469821-0b4fd0a0-74e5-464a-93dd-b196089ea772.png)


### Ubuntu
```bash
$ sudo apt update
$ sudo apt upgrade
$ sudo apt -y install xubuntu-desktop
```

### ROS
```bash
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt -y install curl # if you haven't already installed curl
$ curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
$ sudo apt update
$ sudo apt -y install ros-melodic-desktop-full
$ echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
$ source /opt/ros/melodic/setup.bash
$ sudo apt -y install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
$ sudo apt -y install python-rosdep
$ sudo rosdep init
$ rosdep update
```
参考：http://wiki.ros.org/melodic/Installation/Ubuntu

### pip3
```bash
$ sudo apt install python3-pip  
```
もしエラーが起きれば以下のサイトを参照する
- 前回電源を切ったときに何かしら不具合が生じ、sudo apt 系ができなくなる現象
https://github.com/ShunjiHashimoto/tang/issues/4#issuecomment-907743709
- ROSの署名が古い
https://github.com/ShunjiHashimoto/tang/issues/4#issuecomment-907744118

### Opencv
```bash
$ sudo apt update 
$ sudo apt -y upgrade
$ sudo apt -y install python-pip
$ pip install --upgrade pip
$ pip3 install -U pip
$ pip3 install opencv-python
$ pip3 install opencv-contrib-python
$ pip3 install rospkg catkin_pkg
```
参考：https://github.com/ShunjiHashimoto/tang/issues/8

### camera
```bash
$ sudo apt-get -y install ros-melodic-uvc-camera  
$ sudo apt-get -y install ros-melodic-image-*  
```
### joystick
```bash
$ sudo apt-get -y install ros-melodic-joy
```
### Raspberry Pi 4
```bash
$ sudo apt-get -y install python-rpi.gpio
```

## 実行
$ ssh ubuntu@raspi.local  
パスワード入力  
$ source ~/.bashrc  
$ roslaunch tang_bringup tang_bringup.launch  
