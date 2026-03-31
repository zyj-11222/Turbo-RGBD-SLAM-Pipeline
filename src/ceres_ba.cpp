#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <ceres/ceres.h>
#include <ceres/rotation.h>

using namespace std;
using namespace cv;

// ==========================================================
// Ceres 代价函数 1：点在第一张照片的“重投影误差”
// ==========================================================
struct Cam1Error {
    Cam1Error(double ox, double oy, double fx, Point2d pp) : ox(ox), oy(oy), f(fx), pp(pp) {}
    template <typename T>
    bool operator()(const T* const pt, T* residual) const {
        // 第一张照片在原点，直接根据 3D 点算投影像素
        T px = T(f) * (pt[0] / pt[2]) + T(pp.x);
        T py = T(f) * (pt[1] / pt[2]) + T(pp.y);
        residual[0] = px - T(ox); // X轴误差
        residual[1] = py - T(oy); // Y轴误差
        return true;
    }
    double ox, oy, f; Point2d pp;
};

// ==========================================================
// Ceres 代价函数 2：点在第二张照片的“重投影误差”
// ==========================================================
struct Cam2Error {
    Cam2Error(double ox, double oy, double fx, Point2d pp, Mat R, Mat t) 
        : ox(ox), oy(oy), focal(fx), pp_(pp) {
        // OpenCV 旋转矩阵转 Ceres 旋转向量
        Mat rvec; Rodrigues(R, rvec); 
        cam_r[0] = rvec.at<double>(0,0); cam_r[1] = rvec.at<double>(1,0); cam_r[2] = rvec.at<double>(2,0);
        cam_t[0] = t.at<double>(0,0); cam_t[1] = t.at<double>(1,0); cam_t[2] = t.at<double>(2,0);
    }
    template <typename T>
    bool operator()(const T* const point_3d, T* residual) const {
        T p_w[3] = {point_3d[0], point_3d[1], point_3d[2]};
        T p_c[3];
        T c_r[3] = {T(cam_r[0]), T(cam_r[1]), T(cam_r[2])};
        // 核心数学：Pc = R * Pw + t
        ceres::AngleAxisRotatePoint(c_r, p_w, p_c); 
        p_c[0] += T(cam_t[0]); p_c[1] += T(cam_t[1]); p_c[2] += T(cam_t[2]); 
        
        T px = T(focal) * (p_c[0] / p_c[2]) + T(pp_.x);
        T py = T(focal) * (p_c[1] / p_c[2]) + T(pp_.y);
        residual[0] = px - T(ox);
        residual[1] = py - T(oy);
        return true;
    }
    double ox, oy, focal; Point2d pp_;
    double cam_r[3], cam_t[3];
};

int main() {
    Mat img_1 = imread("1.png", IMREAD_COLOR);
    Mat img_2 = imread("2.png", IMREAD_COLOR);
    if (img_1.empty()) return -1;

    // 1. OpenCV 前端：提取与匹配 (速通版)
    vector<KeyPoint> kp1, kp2; Mat desc1, desc2;
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
    detector->detectAndCompute(img_1, noArray(), kp1, desc1);
    detector->detectAndCompute(img_2, noArray(), kp2, desc2);

    vector<DMatch> matches; matcher->match(desc1, desc2, matches);
    double min_dist = 10000;
    for (int i = 0; i < desc1.rows; i++) if (matches[i].distance < min_dist) min_dist = matches[i].distance;
    
    vector<Point2f> pts1, pts2;
    for (int i = 0; i < desc1.rows; i++) {
        if (matches[i].distance <= max(2 * min_dist, 30.0)) {
            pts1.push_back(kp1[matches[i].queryIdx].pt);
            pts2.push_back(kp2[matches[i].trainIdx].pt);
        }
    }

    // 2. 算位姿
    double focal_length = img_1.cols * 1.2;
    Point2d principal_point(img_1.cols / 2.0, img_1.rows / 2.0);
    Mat E, mask, R, t;
    E = findEssentialMat(pts1, pts2, focal_length, principal_point, RANSAC, 0.999, 1.0, mask);
    recoverPose(E, pts1, pts2, R, t, focal_length, principal_point, mask);

    // 提取纯净内点
    vector<Point2f> in1, in2;
    for (int i = 0; i < mask.rows; i++) {
        if (mask.at<uchar>(i, 0)) { in1.push_back(pts1[i]); in2.push_back(pts2[i]); }
    }

    // ==========================================================
    // 3. Ceres Bundle Adjustment (最燃的时刻)
    // ==========================================================
    ceres::Problem problem;
    // 分配内存来存储 3D 点
    vector<vector<double>> points_3d(in1.size(), vector<double>(3));

    for (size_t i = 0; i < in1.size(); i++) {
        // 瞎猜初始化：强制设定深度 Z = 1.0 (放弃线性代数，让优化器来教它做人)
        points_3d[i][2] = 1.0; 
        points_3d[i][0] = (in1[i].x - principal_point.x) / focal_length * points_3d[i][2];
        points_3d[i][1] = (in1[i].y - principal_point.y) / focal_length * points_3d[i][2];

        // 告诉 Ceres：去缩小第一张照片的误差！
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<Cam1Error, 2, 3>(
                new Cam1Error(in1[i].x, in1[i].y, focal_length, principal_point)),
            nullptr, points_3d[i].data());

        // 告诉 Ceres：同时去缩小第二张照片的误差！
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<Cam2Error, 2, 3>(
                new Cam2Error(in2[i].x, in2[i].y, focal_length, principal_point, R, t)),
            nullptr, points_3d[i].data());
    }

    // 启动求解器！
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    
    cout << "\n🚀 开始 Ceres 暴力优化三角测量...\n";
    ceres::Solve(options, &problem, &summary);
    cout << summary.BriefReport() << "\n\n";

    cout << "🎉 优化完成！前 5 个内点的绝对真实深度 (Z):" << endl;
    for (int i = 0; i < min(5, (int)points_3d.size()); i++) {
        cout << "Point " << i << " depth: " << points_3d[i][2] << endl;
    }

    return 0;
}
