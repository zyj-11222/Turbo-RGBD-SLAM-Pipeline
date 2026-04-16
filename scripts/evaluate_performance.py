#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import subprocess
import argparse

def main():
    print("🚀 [Turbo-SLAM] 自动化评测流水线启动...")
    
    # 1. 定义文件路径 (请确保这两个 txt 文件存在于该目录下)
    dataset_dir = "/home/zyj/slam_ws/src/my_slam/sequence/"
    gt_file = os.path.join(dataset_dir, "groundtruth.txt")
    estimate_file = os.path.join(dataset_dir, "camera_poses.txt")
    
    # 检查文件是否存在
    if not os.path.exists(gt_file):
        print(f"🚨 错误：找不到真值文件 {gt_file}")
        return
    if not os.path.exists(estimate_file):
        print(f"🚨 错误：找不到估计轨迹文件 {estimate_file}")
        return

    # 输出目录设置
    output_dir = "/home/zyj/slam_ws/src/my_slam/results"
    os.makedirs(output_dir, exist_ok=True)
    plot_file = os.path.join(output_dir, "trajectory_ape.png")
    result_zip = os.path.join(output_dir, "metrics.zip")

    print(f"📊 正在对比真值与估计轨迹...")
    print(f"  - 真值: {gt_file}")
    print(f"  - 估计: {estimate_file}")

    # 2. 构建 evo 评测命令
    # evo_ape: 计算绝对位姿误差 (Absolute Pose Error)
    # tum: 指定文件格式为 TUM 格式
    # -a: (align) 自动进行 SE(3) 轨迹对齐，因为我们的 VO 起点可能和真实起点不同
    # -p: (plot) 生成图表
    # --save_plot / --save_results: 后台自动保存，不弹窗阻塞流水线
    cmd = [
        "evo_ape", "tum", gt_file, estimate_file,
        "-a", 
        "--save_plot", plot_file,
        "--save_results", result_zip
    ]

    # 3. 执行系统命令并捕获输出
    try:
        # 运行命令，等待完成
        process = subprocess.run(cmd, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        print("\n✅ 评测完成！Evo 输出信息如下:")
        print("-" * 40)
        # 过滤输出，只打印关键的误差数据 (RMSE, Mean, Max)
        for line in process.stdout.split('\n'):
            if "rmse" in line or "mean" in line or "max" in line or "min" in line:
                print(line.strip())
        print("-" * 40)
        
        print(f"📁 误差曲线图已保存至: {plot_file}")
        print(f"📁 详细数据包已保存至: {result_zip}")
        
    except subprocess.CalledProcessError as e:
        print(f"🚨 评测失败，请检查 evo 是否正确安装或轨迹格式是否正确。")
        print(e.stderr)

if __name__ == '__main__':
    main()
