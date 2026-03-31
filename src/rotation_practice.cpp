#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry> // 旋转转换都在这个头文件里

int main() {
    // 1. 定义一个旋转向量：绕 Z 轴旋转 45 度 (M_PI/4)
    // 方向即轴 (0,0,1)，长度即角度
    Eigen::AngleAxisd rotation_vector(M_PI / 4, Eigen::Vector3d::UnitZ());
    std::cout << "Rotation Vector angle is: " << rotation_vector.angle() << std::endl;

    // 2. 旋转向量 -> 旋转矩阵
    Eigen::Matrix3d rotation_matrix = rotation_vector.toRotationMatrix();

    // 3. 旋转向量 -> 四元数 (面试重点！)
    Eigen::Quaterniond q = Eigen::Quaterniond(rotation_vector);
    std::cout << "Quaternion coefficients (x, y, z, w):\n" << q.coeffs() << std::endl;
    // 注意：Eigen 的 coeffs 顺序是 (x, y, z, w)，实部在最后！

    // 4. 四元数 -> 旋转矩阵
    Eigen::Matrix3d R_from_q = q.toRotationMatrix();
    // 验证任务 1：计算数学意义上的旋转向量 phi = theta * n
    Eigen::Vector3d phi = rotation_vector.angle() * rotation_vector.axis();
    std::cout << "-----------------------------------" << std::endl;
    std::cout << "The mathematical Rotation Vector (phi = theta * n) is:\n" << phi << std::endl;
    std::cout << "Check: The length of this vector should be PI/4 (" << M_PI/4 << ")" << std::endl;
    return 0;
}
