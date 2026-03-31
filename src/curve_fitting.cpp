#include <iostream>
#include <vector>
#include <random>
#include <ceres/ceres.h>

// ==========================================================
// 1. 定义代价函数 (Cost Function) 的模型
// 也就是告诉 Ceres：误差 (残差) 是怎么算出来的？
// ==========================================================
struct CurveFittingCost {
    CurveFittingCost(double x, double y) : _x(x), _y(y) {}

    // 残差的计算逻辑 (必须写成模板函数，因为 Ceres 内部要用自动求导)
    template <typename T>
    bool operator()(const T* const abc, T* residual) const {
        // 模型: y_predict = exp(a * x^2 + b * x + c)
        // abc[0] 是 a, abc[1] 是 b, abc[2] 是 c
        T y_predict = ceres::exp(abc[0] * T(_x) * T(_x) + abc[1] * T(_x) + abc[2]);
        
        // 残差 = 测量值 - 预测值
        residual[0] = T(_y) - y_predict;
        return true;
    }

    const double _x, _y; // 存放传感器观测到的数据点
};

int main() {
    // 真实参数
    double a = 1.0, b = 2.0, c = 1.0; 
    int N = 100; // 观测数据点数量
    double w_sigma = 0.5; // 噪声标准差 (数值越大，数据越烂)
    std::default_random_engine generator;
    std::normal_distribution<double> noise(0.0, w_sigma);

    // 1. 生成带噪声的观测数据 (模拟烂传感器的测量结果)
    std::vector<double> x_data, y_data;
    for (int i = 0; i < N; i++) {
        double x = i / 100.0;
        x_data.push_back(x);
        y_data.push_back(exp(a * x * x + b * x + c) + noise(generator));
    }

    // 2. 构建 Ceres 优化问题
    double abc[3] = {0.0, 0.0, 0.0}; // 这是我们的初始瞎猜值
    ceres::Problem problem;

    for (int i = 0; i < N; i++) {
        // 将每一个观测点，作为一个“误差项”加入到问题中
        problem.AddResidualBlock(
            // 使用自动求导机制 (参数1: 我们的代价函数结构体, 参数2: 输出残差的维度(1维), 参数3: 输入参数的维度(3个))
            new ceres::AutoDiffCostFunction<CurveFittingCost, 1, 3>(
                new CurveFittingCost(x_data[i], y_data[i])
            ),
            nullptr, // 核函数 (我们暂时不用，填空)
            abc      // 待优化的参数块
        );
    }

    // 3. 配置求解器 (Solver)
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR; // 求解线性方程组的方法
    options.minimizer_progress_to_stdout = true;  // 在终端打印优化的神奇过程

    ceres::Solver::Summary summary;

    std::cout << "开始疯狂优化中...\n";
    // 4. 见证奇迹的时刻：开始求解！
    ceres::Solve(options, &problem, &summary);

    // 5. 输出结果
    std::cout << summary.BriefReport() << "\n";
    std::cout << "真实参数: a=1.0, b=2.0, c=1.0\n";
    std::cout << "Ceres 优化反推出的参数: " 
              << "a=" << abc[0] << ", b=" << abc[1] << ", c=" << abc[2] << "\n";

    return 0;
}
