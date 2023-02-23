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

## SetUp
### 制御用PC - Raspberry Pi 4
#### Install Ubuntu
Ubuntuが入ったPCでUbuntu serverをダウンロード
```bash
$ wget http://cdimage.ubuntu.com/releases/bionic/release/ubuntu-18.04.5-preinstalled-server-arm64+raspi3.img.xz
```
ファイルを解凍
```bash
$ xz -dv ubuntu-18.04.5-preinstalled-server-arm64+raspi3.img.xz 
```
Raspberry Pi Imagerを使って書き込む  
![Screenshot from 2021-10-08 07-17-44](https://user-images.githubusercontent.com/63869336/136469821-0b4fd0a0-74e5-464a-93dd-b196089ea772.png)  
書き込み後、Raspberry Piを起動し、xubuntuをinstallする
```bash
$ sudo apt update
$ sudo apt upgrade
$ sudo apt -y install xubuntu-desktop
```

#### Install ROS
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

#### Install Pip
```bash
$ sudo apt install python3-pip  
```
もしエラーが起きれば以下のサイトを参照する
- 前回電源を切ったときに何かしら不具合が生じ、sudo apt 系ができなくなる現象
https://github.com/ShunjiHashimoto/tang/issues/4#issuecomment-907743709
- ROSの署名が古い
https://github.com/ShunjiHashimoto/tang/issues/4#issuecomment-907744118

#### Install Opencv
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

#### Install RPI.GPIO
```bash
$ sudo apt-get -y install python-rpi.gpio
```

### 画像処理用PC - Jetson Xavier
TODO

## 実行方法
TODO

## ライセンス
[Apache License, Version 2.0](https://www.apache.org/licenses/LICENSE-2.0)
