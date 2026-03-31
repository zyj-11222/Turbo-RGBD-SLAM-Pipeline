#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h> // 引入滤波头文件
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;

int main() {
    // 1. 创建三朵云：帧A, 帧B, 以及最终的地图 Map
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_A(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_B(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr world_map(new pcl::PointCloud<pcl::PointXYZ>);

    // 模拟：从文件读取或随机生成
    // 这里我们生成一个简单的“L型墙角”
    for (float i = 0; i < 5.0; i += 0.1) {
        cloud_A->push_back(pcl::PointXYZ(i, 0, 0)); // 墙面1
        cloud_A->push_back(pcl::PointXYZ(0, i, 0)); // 墙面2
    }

    // 模拟第二帧：平移了一点，且带点噪声
    *cloud_B = *cloud_A;
    for (auto& p : *cloud_B) { p.x += 0.5f; p.y += 0.2f; p.z += 0.1f; }

    // 2. 执行 ICP 获取变换矩阵 T
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud_B);
    icp.setInputTarget(cloud_A);
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_B(new pcl::PointCloud<pcl::PointXYZ>);
    icp.align(*aligned_B);

    // 3. 【缝合】把第一帧和对齐后的第二帧合并
    *world_map = *cloud_A;
    *world_map += *aligned_B; // 简单的加法就是拼接
    cout << "缝合前总点数: " << world_map->size() << endl;

    // 4. 【关键：体素滤波】瘦身处理
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(world_map);
    sor.setLeafSize(0.2f, 0.2f, 0.2f); // 设置 20cm 的立方体格子
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    sor.filter(*cloud_filtered);
    cout << "体素滤波后总点数: " << cloud_filtered->size() << endl;

    // 5. 可视化最终地图
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Global Map"));
    viewer->addPointCloud(cloud_filtered, "map");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "map");
    
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }
    return 0;
}
