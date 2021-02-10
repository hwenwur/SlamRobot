# SlamRobot 使用手册
## 凡例
|术语|解释|
|---|---|
|PC|开发所用电脑，必须是 `Ubuntu18.04`|
|SBC|单板机，装载到机器人上的小电脑，可以是任意支持 Docker 的 amd64 或 arm64 设备|

## 安装
### `PC` 上安装步骤
支持平台：`Ubuntu 18.04` `amd64`, `arm64`
1. 拉取本项目
```shell
cd ~
git clone https://github.com/srm2021/SlamRobot
```
2.  安装 ROS melodic

2.1 安装 ROS 主程序

以下内容摘自官方文档，根据我国网络环境做了修改。原文链接：http://wiki.ros.org/melodic/Installation/Ubuntu

2.1.1 添加软件源
```shell
sudo sh -c 'echo "deb http://mirrors.sjtug.sjtu.edu.cn/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

2.1.2 添加密钥
```shell
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
```

2.1.3 安装程序
```shell
sudo apt install ros-melodic-desktop-full
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo apt install python-rosdep
```

2.1.4 环境配置（非 `bash` 用户请参考官方文档）
```shell
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

2.1.5 初始化 `rosdep`（需要翻墙）。
```shell
sudo rosdep init
rosdep update
```

2.2 安装 ROS package
```shell
cd ~/SlamRobot/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
```
关于 `rosdep` 的用法，参见：http://wiki.ros.org/rosdep

3. 安装调试工具
```shell
sudo apt install ros-melodic-rviz ros-melodic-rqt ros-melodic-teleop-twist-keyboard
```

关于 `rviz` 的用法，参见：http://wiki.ros.org/rviz

### `SBC` 上安装步骤
使用 Docker 的步骤：
```shell
cd ~/SlamRobot
docker build -t hwenwur/ros .
```
不使用 Docker 的步骤：

同 `PC` 端的步骤1,2。

**注意，仿真软件不能在 Docker 中运行。**

## 配置
### `SBC` 上配置步骤
1. 绑定静态串口设备名：
-  将雷达串口设备链接到：`/dev/lidar`
- 将 `stm32` 串口设备链接到：`/dev/stm32`

配置方法参见：https://linux.die.net/man/8/udev

2. 将 WIFI 设为静态获取 IP 地址。（可选）

## 运行仿真环境
1. 下载比赛场地图纸（`.dae` 格式）

由于场地图纸体积较大，不适合放在 git 仓库中。可以在官方论坛（关键词搜索“场地”）下载`.stp`文件，用建模软件导出成`.dae`格式。然后用任意编辑器打开`~/SlamRobot/catkin_ws/src/bot_simulation/worlds/battle_ground.sdf`，搜索关键词`file://`，将后面的路径替换成`.dae`文件的路径（共两处）。注意完成之后 `file:` 后面应有三个斜杠。

2. 启动 `Gazebo`，运行：（如果终端用的是 `zsh` 而不是 `bash` 需要将`source ./devel/setup.bash`改成`source ./devel/setup.zsh`。下同）
```shell
cd ~/SlamRobot/catkin_ws
source ./devel/setup.bash
roslaunch bot_simulation simulate.launch
```
此时可以看到 `3D` 渲染的场景和机器人的窗口。

3. 启动建图程序
```shell
cd ~/SlamRobot/catkin_ws
source ./devel/setup.bash
roslaunch cartographer_mapping gazebo_mapping.launch
```
此时可以看到 `rviz` 窗口，其中有实时的建图状态和其他必要的调试信息。

4. 启动键盘控制器
```shell
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
此时可以通过键盘控制仿真环境中的机器人运动。

5. 启动导航算法（待完善）
```shell
cd ~/SlamRobot/catkin_ws
source ./devel/setup.bash
roslaunch robot_navigation move_base.launch
```
此时机器人可以在 `rviz` 中指定目标点后自动规划路线运动。

6. 全自动运行
待开发...

## 在真实环境运行
(未完待续...)
