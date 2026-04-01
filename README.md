# Mini SLAM System: 视觉与激光里程计实战

本项目是一个基于 C++17 开发的轻量级 SLAM（同时定位与建图）算法实践库。涵盖了从单目视觉里程计 (VO) 前端、后端非线性优化 (BA) 到激光雷达点云配准 (ICP) 的核心流程。

## 🛠️ 技术栈
* **语言**: C++17, CMake
* **视觉算法**: OpenCV 4 (ORB特征, RANSAC, 对极几何, 三角测量)
* **后端优化**: Ceres Solver (Bundle Adjustment, 重投影误差优化)
* **激光算法**: PCL 1.2+ (ICP, VoxelGrid 降采样, 3D 可视化)

## 🚀 核心模块与功能

### 1. 视觉前端 (Visual Odometry)
* `src/vo_sequence.cpp`: 基于 TUM 数据集 (`fr1/desk`) 实现了 Frame-to-Frame 的单目视觉里程计。
* **亮点**: 深入解决了相机纯旋转与共面场景下的三角化数值下溢出 (Numerical Underflow) 问题，实现了基于归一化平面的高鲁棒性位姿求解。

### 2. 后端优化 (Backend Optimization)
* `src/ceres_ba.cpp`: 构建了基于 Ceres 的非线性图优化器。
* **亮点**: 通过自动求导 (AutoDiff) 实现了 Structure-Only Bundle Adjustment，最小化重投影误差，并在工程中深刻验证了单目 SLAM 的尺度漂移 (Scale Drift) 现象。

### 3. 激光点云配准与建图 (LiDAR Mapping)
* `src/icp_test.cpp` & `pcl_visualize.cpp`: 手动拆解并实现了 ICP (迭代最近点) 算法的 3D 动态可视化。
* `src/pcl_mapping.cpp`: 实现了多帧点云的坐标系增量拼接，并引入 Voxel Grid 体素滤波解决了大规模点云建图的内存爆炸问题。
### 📈 精度评估 (Accuracy Evaluation)
本项目使用 `evo` 工具在 TUM RGB-D 数据集上进行了闭环测试。

**测试序列**：`fr1/desk`
**评估指标**：Absolute Trajectory Error (ATE)

![Trajectory Plot](./docs/evo_result.png)
*注：当前版本为纯单目里程计实现，未加入尺度闭环，图中展示了经过 Sim(3) 对齐后的轨迹形态，RMSE 表现符合单目 VO 预期。*
## 🏃 如何运行

```bash
mkdir build && cd build
cmake ..
make
# 运行视觉里程计 (请确保已下载 TUM 数据集至正确路径)
./vo_sequence
# 运行点云缝合建图
./pcl_mapping
