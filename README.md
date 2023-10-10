# 収穫サポートロボット「TANG」
[CuGo V3](https://cuborex.com/cugo)を使った収穫サポートロボットTANG 
<img width="400" src="doc/images/tang.png">

開発背景などの詳細はロボゼミでの[発表スライド](https://docs.google.com/presentation/d/1ee6lWTKakHZedKpi5SkKf4kFQXAP9l1Z/edit#slide=id.p1)をご確認ください

<img width="600" src="doc/images/abst.png">

## 使用した主な部品
- 足回り
  - [CuGo V3](https://cuborex.com/cugo)
- カメラ
  - [Webカメラ](https://amzn.asia/d/0IFd8t9)
    - 視野角 150°
    - 200万画素
- 制御用PC
  - Raspberry Pi 4（Ubuntu 20.04, ROS 1, noetic）
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
必要なPythonのライブラリをインストールするために必要なライブラリです。
```bash
$ sudo apt install python3-pip  
```

#### Install Opencv
OpenCVは画像処理機能をまとめたオープンソースのライブラリです。
今回の場合は、カメラ画像を取得したり、得られた画像をもとに画像処理を行い、色検出を行うために使用します。
```bash
$ sudo apt update 
$ pip3 install opencv-python
$ pip3 install opencv-contrib-python
```

#### Install pigpio
Raspberry PiのGPIOピンを操作するときに使用します。例えば、PWM制御やスイッチの入出力などに使用します。
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
TANGを遠隔操作するときに使用するJoystickをROSで動作させるためのパッケージをインストールします。
```bash
$ sudo apt-get install ros-noetic-joy
```

## パッケージ構成
```
├── ./doc ## READMEに記載する画像など
├── ./tang_bringup # ロボットを起動するためのパッケージ  
│   └── ./tang_bringup/launch
├── ./tang_control # ロボット制御用のパッケージ
│   └── ./tang_control/scripts
│       └── ./tang_control/scripts/mymodule
├── ./tang_detection # 色検出用のパッケージ
│   ├── ./tang_detection/launch
│   └── ./tang_detection/scripts
├── ./tang_msgs # ロボットの通信用のパッケージ
│   └── ./tang_msgs/msg
├── ./tang_teleop # ロボットの遠隔操作用パッケージ  
│   ├── ./tang_teleop/launch
│   └── ./tang_teleop/scripts
└── ./test ## エンコーダ値の取得や速度制御のテスト用スクリプト
```
### tang_bringup
ロボットを起動するためのパッケージ  
tang_detectionとtang_control 2つのパッケージを同時に起動する。  
### tang_control
ロボット制御用のパッケージ  
色検出結果をもとに対象に追従する。また、モード切替をすることで、joystickを使ってロボットの遠隔操作も行う。  
##### パラメータ調整（ゲイン調整、速度設定）
/tang_control/config.pyでパラメータ調整が可能です。  
PID制御のゲイン調整
```bash
# 人追従時のPID
class FOLLOWPID:
    p_gain = 0.002
    d_gain = 0.0
    #d_gain = 7.0
    dt = 0.01

# 速度制御時のPID
class PID:
    Kp = 1.0
    Ki = 0.01
    Kd = 0.00
```
最大速度、最大角速度、目標加速度、目標減速加速度の設定を行います。

```bash
# 最大角速度
max_w = 0.5
# 最大速度
max_velocity = 0.2
# 目標加速度
a_target = 0.01
# 目標減速加速度
d_target = -0.01
``` 
### tang_detection
色検出用のパッケージ  
カメラから取得した画像をもとに、予め設定された色を検出し、tang_controlノードに検出結果を送る。  
##### パラメータ調整（色検出の閾値設定）
tang_detection/scripts/color_object_detection.py内の下記のcalc_mask関数で設定する。  
```bash
def calc_mask(self, hsv):
    # 緑色のHSVの値域
    hsv_min = np.array([30,120,0]) 
    hsv_max = np.array([80,255,255])
    # 緑色領域のマスク
    mask = cv2.inRange(hsv, hsv_min, hsv_max)  
    return(mask)
```
### tang_msgs
tang_control, tang_detection, tang_teleopそれぞれのパッケージで実行されたノード同士が通信するために必要なメッセージの定義
### tang_teleop
ロボットの遠隔操作用パッケージ  
Joystickから受信した信号をもとに、ロボットを遠隔操作する。  
##### パラメータ調整（遠隔操作時の速度設定）
JoystickのAボタンを押すと速度が上がり、Bボタンを押すと速度が下がります。  
また、JoystickのBackボタンを押すと遠隔操作と人追従のモードを切り替えられます。
## 実行方法
#### 色検出＋ロボットを起動する
色検出を行い、ロボットが対象を追従します。
```bash
$ roslaunch tang_bringup tang_bringup.launch
```
#### 色検出のみを起動する場合
ロボットを動作させずに、色検出のみを実行したい場合は、下記の通りに実行してください。主に、色検出時の色の閾値の調整時に実行します。
```bash
$ roslaunch tang_detection tang_detection.launch
```

## ライセンス
[Apache License, Version 2.0](https://www.apache.org/licenses/LICENSE-2.0)
