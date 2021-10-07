# 収穫サポートロボット「TANG」
CuGo V3を使って収穫サポートロボットTanGを作る

## 必要な開発環境
- Rapberry Pi 4
- Ubuntu18.04が入ったPC
- Webカメラ
- CuGo V3 など

## 必要なパッケージ
### Ubuntu
```bash
sudo apt update
sudo apt upgrade
sudo apt install xubuntu-desktop
```

### ROS
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-melodic-desktop-full
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
source /opt/ros/melodic/setup.bash
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo apt install python-rosdep
sudo rosdep init
rosdep update

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
sudo apt install python-pip
pip install --upgrade pip
pip3 install -U pip
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
