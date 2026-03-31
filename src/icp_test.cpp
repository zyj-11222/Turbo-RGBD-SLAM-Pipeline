#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

using namespace std;

int main (int argc, char** argv)
{
    // 1. 准备两朵云：cloud_in (原始云) 和 cloud_out (目标云)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>(50, 1));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);

    // 2. 随机生成 50 个 3D 坐标点塞进原始云
    for (auto& point : *cloud_in) {
        point.x = 1024 * rand() / (RAND_MAX + 1.0f);
        point.y = 1024 * rand() / (RAND_MAX + 1.0f);
        point.z = 1024 * rand() / (RAND_MAX + 1.0f);
    }
    cout << "✅ 成功生成 50 个随机 3D 点。" << endl;

    // 3. 把原始云复制一份给目标云
    *cloud_out = *cloud_in;

    // 4. 【上帝视角】我们手动把目标云平移一下：X轴移动 0.7米，Z轴移动 1.5米
    for (auto& point : *cloud_out) {
        point.x += 0.7f;
        point.z += 1.5f;
    }
    cout << "😈 我们偷偷把目标云平移了: X +0.7, Z +1.5" << endl;

    // ==========================================
    // 5. 见证奇迹的时刻：启动 ICP 算法！
    // ==========================================
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    
    // 告诉 ICP 谁是起点，谁是终点
    icp.setInputSource(cloud_in);
    icp.setInputTarget(cloud_out);
    
    // 这个 Final 云是用来存放对齐后的结果的
    pcl::PointCloud<pcl::PointXYZ> Final;
    
    cout << "\n🚀 开始执行 ICP 对齐..." << endl;
    icp.align(Final);

    // 6. 打印成绩单
    cout << "是否成功收敛 (Converged)? " << (icp.hasConverged() ? "是 (Yes)" : "否 (No)") << endl;
    cout << "最终匹配均方误差 (Fitness Score): " << icp.getFitnessScore() << endl;
    
    // 7. 打印 ICP 算出来的 4x4 变换矩阵 [R | t]
    // 左上角 3x3 是旋转 R，最右边一列是平移 t！
    cout << "\n🎯 ICP 反推出来的变换矩阵:" << endl;
    cout << icp.getFinalTransformation() << endl;

    return 0;
}
