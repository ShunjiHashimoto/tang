# 収穫サポートロボット「TANG」
[CuGo V3](https://cuborex.com/cugo)を使った収穫サポートロボットTANG  
開発背景などの詳細はロボゼミでの[発表スライド](https://docs.google.com/presentation/d/1ee6lWTKakHZedKpi5SkKf4kFQXAP9l1Z/edit#slide=id.p1)をご確認ください 

<img width="400" src="doc/images/tang.png">

## 使用した主な部品
- 足回り
  - [CuGo V3](https://cuborex.com/cugo)
- カメラ
  - Webカメラ
- 制御用PC
  - Raspberry Pi 4（Ubuntu 20.04, ROS 1）
- IMU
  - [WT901C-RS232](https://github.com/witmotion/WT901C-RS232)
- モータドライバ
  - [Cytron SmartDriveDuo 10A](https://www.cytron.io/p-10amp-5v-30v-dc-motor-driver-2-channels)
- 電源
  - モータ駆動用電源 24V
  - 制御用電源
    - モバイルバッテリ（5V, 3A出力）

## SetUp
### 【制御用PC】Raspberry Pi 4
#### Install Ubuntu
Ubuntuが入ったPCで[rpi_xubntu_ros](https://github.com/Ar-Ray-code/rpi_xubuntu_ros)を参照しながら、Xubuntu + ROSの[イメージファイル](https://github.com/Ar-Ray-code/rpi_xubuntu_ros/releases/tag/20.04_v1.0)をダウンロードした後、[Raspberry Pi Imager](https://www.raspberrypi.com/software/)を使って書き込む  


#### Install ROS
```bash
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt -y install curl 
$ curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
# install ROS
$ sudo apt update
$ sudo apt -y install ros-noetic-desktop-full
# パス設定
$ echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
$ source /opt/ros/noetic/setup.bash
# rosdep
$ sudo apt -y install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
$ sudo rosdep init
$ rosdep update
# catkin pkg
$ pip3 install rospkg catkin_pkg
```

#### Install Pip
```bash
$ sudo apt install python3-pip  
```

#### Install Opencv
```bash
$ sudo apt update 
$ pip3 install opencv-python
$ pip3 install opencv-contrib-python
```

#### Install pigpio
```bash
$ sudo apt update
$ sudo apt install build-essential
$ git clone https://github.com/joan2937/pigpio
$ cd pigpio
$ make
$ sudo make install
# pigpio daemonを起動する
$ sudo pigpiod
```

#### Install Joy
```bash
$ sudo apt-get install ros-noetic-joy
```

## 実行方法
TODO

## ライセンス
[Apache License, Version 2.0](https://www.apache.org/licenses/LICENSE-2.0)
