#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>

int main() {
    // 1. 初始化点云（红色为 Source，绿色为 Target）
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZ>(100, 1));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZ>);
    
    for (auto& p : *cloud_source) {
        p.x = 5.0f * rand() / (RAND_MAX + 1.0f);
        p.y = 5.0f * rand() / (RAND_MAX + 1.0f);
        p.z = 5.0f * rand() / (RAND_MAX + 1.0f);
    }
    *cloud_target = *cloud_source;
    for (auto& p : *cloud_target) { p.x += 1.0f; p.y += 0.5f; } // 制造位移

    // 2. 创建可视化窗口
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("ICP Animation"));
    viewer->setBackgroundColor(0, 0, 0);
    
    // 添加 Target (绿色) - 固定不动
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(cloud_target, 0, 255, 0);
    viewer->addPointCloud(cloud_target, target_color, "target");

    // 添加 Source (红色) - 它是会动的
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_color(cloud_source, 255, 0, 0);
    viewer->addPointCloud(cloud_source, source_color, "source");

    // 3. 配置 ICP
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud_source);
    icp.setInputTarget(cloud_target);
    // 【关键点】每次只迭代 1 次
    icp.setMaximumIterations(1); 

    std::cout << "点击窗口，按 'Space' 开始吸附动画..." << std::endl;

    // 4. 动画主循环
    while (!viewer->wasStopped()) {
        // 执行 1 次迭代
        pcl::PointCloud<pcl::PointXYZ> tmp;
        icp.align(tmp); 

        if (icp.hasConverged()) {
            // 更新 Source 云的数据
            *cloud_source = tmp; 
            // 告诉 ICP：你的输入源现在是挪动后的新位置了
            icp.setInputSource(cloud_source); 

            // 【关键点】更新可视化界面
            viewer->updatePointCloud(cloud_source, source_color, "source");
        }

        viewer->spinOnce(100); // 这里的 100ms 决定了“慢动作”的速度
    }
    return 0;
}
