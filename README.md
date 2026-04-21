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

## 🛠️ 快速开始 (Quick Start)

### 方式一：本地源码编译 (Local Build)

1. **环境准备**：Ubuntu 20.04 + ROS Noetic + PCL + OpenCV
2. **克隆与编译**：
   ```bash
   mkdir -p ~/slam_ws/src && cd ~/slam_ws/src
   git clone [https://github.com/zyj-11222/Turbo-RGBD-SLAM-Pipeline.git](https://github.com/zyj-11222/Turbo-RGBD-SLAM-Pipeline.git) my_slam
   cd ~/slam_ws
   catkin_make -DCMAKE_BUILD_TYPE=Release
