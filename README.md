# 🚀 Turbo-RGBD-SLAM-Pipeline

![ROS Version](https://img.shields.io/badge/ROS-Noetic-green.svg)
![C++](https://img.shields.io/badge/C++-14-blue.svg)
![License](https://img.shields.io/badge/License-MIT-yellow.svg)
![Build](https://img.shields.io/badge/Build-Passing-brightgreen.svg)

**Turbo-RGBD-SLAM** 是一套面向工业级落地的 RGB-D SLAM 稠密建图与评测流水线。本项目摒弃了传统的“玩具级”脚本写法，全面采用 **面向对象 (OOP)** 设计、**TF 动态坐标树** 管理、**多传感器硬件级时间同步 (Message Filters)**，并通过 **OpenMP 多线程** 将点云处理性能榨取到极致。

---

## ✨ 核心工程特性 (Core Features)

- 🏗️ **解耦式架构设计**：采用 YAML + Launch 的参数服务器架构，算法核心代码与超参数完美隔离，支持零代码修改适配多款传感器。
- ⏱️ **严格的时空同步**：采用 `message_filters::ApproximateTime` 解决 RGB 与 Depth 相机的硬件时钟不同步现象，彻底杜绝“点云拖影/撕裂”。
- ⚡ **OpenMP 极限压榨**：摒弃低效的降采样跳步，利用 C++ 多线程重构双重循环，实现全分辨率点云的毫秒级极速渲染。
- 📊 **CI/CD 级自动化评测**：内置基于 `evo` 的自动化评测脚本，一键生成绝对轨迹误差 (ATE) 的可视化曲线，用数据量化算法精度。
- 🐳 **云原生级一键部署**：提供完整工业级 `Dockerfile`，将繁杂的 PCL/OpenCV/ROS 依赖整体打包，实现“拉取即运行”。

---

# 🚀 Turbo-RGBD-SLAM-Pipeline 启动与运行指南

本项目提供两种运行模式：**本地开发模式**（适合高频修改代码）与 **Docker 部署模式**（适合快速复现环境）。

---

## 🏗️ 准备工作 (Prerequisites)

在开始之前，请确保你已经准备好了数据：

1. **下载数据集**：前往 [TUM RGB-D Dataset](https://www.google.com/search?q=https://vision.in.tum.de/data/datasets/rgbd-dataset/download) 下载 `freiburg1_desk` 序列。
  
2. **存放路径**：建议将数据集解压至 `~/slam_ws/src/my_slam/sequence/` 目录下，并确保包含 `rgb/`, `depth/` 文件夹及 `groundtruth.txt`。
  

---

## 🛠️ 方式一：本地原生编译运行 (Local Native Mode)

**适用场景：** 需要频繁修改 C++ 代码、进行断点调试。

### 1. 环境依赖

- Ubuntu 20.04 + ROS Noetic (Desktop Full)
  
- PCL 1.10+, OpenCV 4.2+
  
- OpenMP (系统自带)
  

### 2. 编译流程

Bash

```
# 进入工作空间
cd ~/slam_ws

# 执行 Release 优化编译（开启 -O3 加速）
catkin_make -DCMAKE_BUILD_TYPE=Release

# 刷新环境变量
source devel/setup.bash
```

### 3. 启动节点

Bash

```
# 一键拉起数据播放、TF广播、前端VO与后端建图
roslaunch my_slam slam_run.launch
```

---

## 🐳 方式二：Docker 容器化一键部署 (Docker Mode)

**适用场景：** 环境快速迁移、在没有安装 ROS 的电脑上运行、或进行 CI/CD 集成。

### 1. 构建镜像

在 `my_slam` 项目根目录下执行：

Bash

```
sudo docker build -t turbo-slam:v1 .
```

### 2. 挂载运行 (终极连招)

通过 **目录映射** 确保数据读写，通过 **Host 网络** 确保与宿主机 RViz 通信：

Bash

```
sudo docker run -it --rm \
  --net=host \
  -v /home/zyj/slam_ws:/home/zyj/slam_ws \
  turbo-slam:v1
```

> **💡 参数解析：**
> 
> - `--net=host`: 允许容器直接使用物理机 IP 端口，物理机执行 `rviz` 即可看到容器内的话题。
>   
> - `-v [宿主机路径]:[容器内路径]`: 目录映射。不仅让容器能读到数据集，还能让容器内生成的 `.pcd` 地图直接保存在物理机硬盘上。
>   

---

## 🖥️ 观测与交互 (Common Operations)

无论采用哪种方式启动，你都可以通过以下操作进行交互：

### 1. 可视化监控 (RViz)

在物理机终端直接输入：

Bash

```
rviz
```

- **Fixed Frame**: 设置为 `map`。
  
- **Add PointCloud2**: 订阅 `/point_cloud_map` 话题。
  
- **Result**: 你将看到 1ms 级响应的全分辨率点云动态生成。
  

### 2. 地图落盘保存

系统运行过程中，随时可以下发保存指令：

Bash

```
rosservice call /save_map
```

地图将以 `my_awesome_desk.pcd` 的文件名保存在项目的根目录下。

### 3. 性能评测 (Evo)

执行自动化评测脚本，查看轨迹误差分析：

Bash

```
python3 scripts/evaluate_performance.py
```
