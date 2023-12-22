# 功能介绍

语音控制小车运动功能通过语音控制机器人向前、向后、向左、向右运动，需要搭配地平线机器人操作系统的智能语音模块一起使用。当用户说出控制机器人运动的指令后，智能语音模块识别到指定，然后下发运动指令给机器人运动。

流程如下图：

![audio_control](./imgs/audio_control.jpg)

该应用可以使用PC端Gazebo仿真环境下的虚拟小车运行，也可以直接用于控制实物小车。

# 机器人实物

## 物料清单

以下机器人均已适配RDK X3

| 机器人名称          | 生产厂家 | 参考链接                                                                                                                                                          |
| :------------------ | -------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| OriginBot智能机器人 | 古月居   | [点击跳转](https://www.originbot.org/)                                                                                                                            |
| X3派机器人          | 轮趣科技 | [点击跳转](https://item.taobao.com/item.htm?spm=a230r.1.14.17.55e556912LPGGx&id=676436236906&ns=1&abbucket=12#detail)                                             |
| 履带智能车          | 微雪电子 | [点击跳转](https://detail.tmall.com/item.htm?abbucket=9&id=696078152772&rn=4d81bea40d392509d4a5153fb2c65a35&spm=a1z10.5-b-s.w4011-22714387486.159.12d33742lJtqRk) |
| RDK X3 Robot        | 亚博智能 | [点击跳转](https://detail.tmall.com/item.htm?id=726857243156&scene=taobao_shop&spm=a1z10.1-b-s.w5003-22651379998.21.421044e12Yqrjm)                               |
| 麦克风板            | 微雪电子 | [点击跳转](https://www.waveshare.net/shop/Audio-Driver-HAT.htm)                                                                                                   |

## 使用方法

### 准备工作

1. 机器人具备运动底盘、相机、环形麦克风板及RDK套件，硬件已经连接并测试完毕；
2. 已有ROS底层驱动，机器人可接收“/cmd_vel”指令运动，并根据指令正确运动。

### 机器人组装

以下操作过程以OriginBot为例，满足条件的其他机器人使用方法类似。参考机器人官网的[使用指引](https://www.originbot.org/guide/quick_guide/)，完成机器人的硬件组装、镜像烧写及示例运行，确认机器人的基础功能可以顺利运行。

### 安装功能包

**1.参考[OriginBot说明](https://github.com/nodehubs/originbot_minimal/blob/develop/README.md)，完成Originbot基础功能安装**

**2.安装功能包**

启动机器人后，通过终端SSH或者VNC连接机器人，复制如下命令在RDK的系统上运行，完成相关Node的安装。

```bash
sudo apt update
sudo apt install -y tros-audio-control
```

### 运行语音控制小车运动功能

**1. 启动机器人底盘**

   启动机器人，如OriginBot的启动命令如下：

   ```bash
   source /opt/tros/setup.bash
   ros2 launch originbot_base robot.launch.py 
   ```

**2. 启动语音控制**

   启动一个新的终端，通过如下指令启动功能：

   ```shell
   # 配置tros.n环境
   source /opt/tros/setup.bash

   # 从地平线RDK的安装路径中拷贝出运行示例需要的配置文件。
   cp -r /opt/tros/lib/hobot_audio/config/ .

   # 屏蔽调式打印信息
   export GLOG_minloglevel=3

   # 启动launch文件
   ros2 launch audio_control audio_control.launch.py
   ```

   启动成功后，当用户说出 "*向前走*" "*向后退*" "*向左转*" "*向右转*" "*停止运动*" 等指令后，机器人按照指令开始运动。

**3. 结果分析**

   地平线RDK运行终端输出如下信息：

   ```shell
         This is audio control package.

   ============================================
         audio control usage

   Wake up device is "地平线你好".
   Audio control commnad word definitions are:
         "向前走": move front.
         "向后退": move back.
         "向右转": rotate robot to right.
         "向左转": rotate robot to left. 
   ============================================

   ```

   以上log截取了一段音频控制pkg启动后的输出。log内容显示，此语音控制模块配置的设备唤醒词是“地平线你好”，控制小车运动的命令词有：“向前走”、“向后退”、“向左转”，“向右转”。

# Gazebo仿真

Gazebo仿真适用于持有RDK X3但没有机器人实物的开发者体验功能。

## 物料清单

| 机器人名称 | 生产厂家 | 参考链接                                                        |
| :--------- | -------- | --------------------------------------------------------------- |
| RDK X3     | 多厂家   | [点击跳转](https://developer.horizon.cc/sunrise)                |
| 麦克风板   | 微雪电子 | [点击跳转](https://www.waveshare.net/shop/Audio-Driver-HAT.htm) |

## 使用方法

### 准备工作

1. 开发者有RDK套件实物，及配套的麦克风板;
2. PC电脑端已经完成ROS Gazebo及Turtlebot机器人相关功能包安装;
3. 和地平线RDK在同一网段（有线或者连接同一无线网，IP地址前三段需保持一致）的PC，PC端需要安装的环境包括：

- Ubuntu 20.04系统

- [ROS2 Foxy桌面版](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)

- Gazebo和Turtlebot3相关的功能包，安装方法：

   ```shell
   sudo apt-get install ros-foxy-gazebo-*
   sudo apt install ros-foxy-turtlebot3
   sudo apt install ros-foxy-turtlebot3-simulations
   ```

### 安装功能包

启动RDK X3后，通过终端SSH或者VNC连接机器人，复制如下命令在RDK的系统上运行，完成相关Node的安装。

```bash
sudo apt update
sudo apt install -y tros-audio-control
```

### 运行功能

**1.启动仿真环境及机器人**

   在PC端Ubuntu的终端中使用如下命令启动Gazebo，并加载机器人模型：

   ```shell
   source /opt/ros/foxy/setup.bash
   export TURTLEBOT3_MODEL=burger
   ros2 launch turtlebot3_gazebo empty_world.launch.py
   ```

   启动成功后，仿真环境中小车效果如下：

   ![gazebo](./imgs/gazebo.jpeg)

**2.启动语音控制**

   启动一个新的终端，通过如下指令启动功能：

   ```shell
   # 配置tros.n环境
   source /opt/tros/setup.bash

   # 从地平线RDK的安装路径中拷贝出运行示例需要的配置文件。
   cp -r /opt/tros/lib/hobot_audio/config/ .

   # 屏蔽调式打印信息
   export GLOG_minloglevel=3

   # 启动launch文件
   ros2 launch audio_control audio_control.launch.py
   ```

   启动成功后，当用户说出 "*向前走*" "*向后退*" "*向左转*" "*向右转*" "*停止运动*" 等指令后，小车按照指令开始运动。

   PC端仿真环境中语音控制小车运动，效果如下：
   ![move](./imgs/move.gif)

# 接口说明

## 话题

| 名称     | 消息类型                | 说明                         |
| -------- | ----------------------- | ---------------------------- |
| /cmd_vel | geometry_msgs/msg/Twist | 发布控制机器人移动的速度指令 |

## 参数

| 参数名                  | 类型        | 解释                                                                                                     | 是否必须 | 支持的配置                                                                                              | 默认值       |
| ----------------------- | ----------- | -------------------------------------------------------------------------------------------------------- | -------- | ------------------------------------------------------------------------------------------------------- | ------------ |
| ai_msg_sub_topic_name   | std::string | 订阅的音频智能帧消息话题                                                                                 | 否       | 根据实际情况配置                                                                                        | /audio_smart |
| twist_pub_topic_name    | std::string | 发布Twist类型的运动控制消息的topic名                                                                     | 否       | 根据实际部署环境配置。一般机器人订阅的topic为/cmd_vel，ROS2 turtlesim示例订阅的topic为turtle1/cmd_vel。 | /cmd_vel     |
| move_step               | float       | 平移运动的步长，单位米                                                                                   | 否       | 无限制                                                                                                  | 0.5          |
| rotate_step             | float       | 旋转运动的步长，单位弧度                                                                                 | 否       | 无限制                                                                                                  | 0.5          |
| motion_duration_seconds | int         | 平移/旋转动作持续时间，单位秒，小于等于0表示不做限制，达到持续时间后下发停止运动指令，避免机器人一直运动 | 否       | 无限制                                                                                                  | 0            |

# 参考资料

语音控制参考：[开发者说 | AI 操控机器人系列第三期 —— 语音控制](https://developer.horizon.cc/forumDetail/109609560406362625)

# 常见问题

1. Ubuntu下运行启动命令报错`-bash: ros2: command not found`

   当前终端未设置ROS2环境，执行命令配置环境：

   ```bash
   source /opt/tros/setup.bash
   ```

   在当前终端执行ros2命令确认当前终端环境是否生效：

   ```shell
   # ros2
   usage: ros2 [-h] Call `ros2 <command> -h` for more detailed usage. ...

   ros2 is an extensible command-line tool for ROS 2.

   optional arguments:
   -h, --help            show this help message and exit
   ```

   如果输出以上信息，说明ros2环境配置成功。

   注意：对于每个新打开的终端，都需要重新设置ROS2环境。

2. 无法打开音频设备

   - 确认音频设备连接是否正常
   - 确认是否正确配置音频设备
   - 确认加载音频驱动前是否已有音频设备连接
