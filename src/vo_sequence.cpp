#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <iomanip>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;
using namespace cv;

// 定义一个结构体来同时存储时间戳和文件名
struct FrameInfo {
    string timestamp;
    string filename;
};

int main() {
    string dataset_dir = "../sequence/";
    string list_file = dataset_dir + "rgb.txt";

    // 1. 初始化保存文件逻辑（放在最前面）
    std::ofstream pose_file("../results/camera_poses.txt");
    if (!pose_file.is_open()) {
        std::cerr << "🚨 无法创建位姿文件！请检查是否已经创建了 results 文件夹。" << std::endl;
        return -1;
    }

    // 2. 读取 rgb.txt 图片清单（包含时间戳）
    ifstream fin(list_file);
    if (!fin) {
        cout << "🚨 找不到 rgb.txt！请检查 sequence 文件夹位置！" << endl;
        return -1;
    }

    vector<FrameInfo> frames;
    string line;
    while (getline(fin, line)) {
        if (line.empty() || line[0] == '#') continue; 
        istringstream iss(line);
        string ts, fname;
        iss >> ts >> fname; 
        frames.push_back({ts, fname});
    }
    cout << "✅ 成功读取图片清单，共 " << frames.size() << " 张图片。" << endl;

    // 3. 初始化全局位姿和画布
    Mat R_global = Mat::eye(3, 3, CV_64F);
    Mat t_global = Mat::zeros(3, 1, CV_64F);
    Mat trajectory = Mat::zeros(600, 600, CV_8UC3);

    Ptr<FeatureDetector> detector = ORB::create(500);
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

    Mat img_prev, desc_prev;
    vector<KeyPoint> kp_prev;

    // TUM 数据集相机内参
    double focal = 517.3; 
    Point2d pp(318.6, 255.3);

    // 窗口排版
    namedWindow("TUM Dataset View", WINDOW_NORMAL);
    resizeWindow("TUM Dataset View", 640, 480);
    namedWindow("Robot Trajectory", WINDOW_NORMAL);
    resizeWindow("Robot Trajectory", 600, 600);

    // 处理第一张图
    img_prev = imread(dataset_dir + frames[0].filename, IMREAD_GRAYSCALE);
    if (img_prev.empty()) { cout << "🚨 读不到第一张图！" << endl; return -1; }
    detector->detectAndCompute(img_prev, noArray(), kp_prev, desc_prev);

    // 4. 开始接力循环
    for (size_t i = 1; i < frames.size(); i++) {
        Mat img_curr = imread(dataset_dir + frames[i].filename, IMREAD_GRAYSCALE);
        if (img_curr.empty()) continue;

        vector<KeyPoint> kp_curr; Mat desc_curr;
        detector->detectAndCompute(img_curr, noArray(), kp_curr, desc_curr);

        vector<DMatch> matches, good_matches;
        matcher->match(desc_prev, desc_curr, matches);
        
        double min_dist = 10000;
        for (int j = 0; j < desc_prev.rows; j++) if (matches[j].distance < min_dist) min_dist = matches[j].distance;
        for (int j = 0; j < desc_prev.rows; j++) {
            if (matches[j].distance <= max(2 * min_dist, 30.0)) good_matches.push_back(matches[j]);
        }

        vector<Point2f> pts_prev, pts_curr;
        for (size_t j = 0; j < good_matches.size(); j++) {
            pts_prev.push_back(kp_prev[good_matches[j].queryIdx].pt);
            pts_curr.push_back(kp_curr[good_matches[j].trainIdx].pt);
        }

        if (pts_prev.size() < 12) continue;

        // 计算相对位姿
        Mat E, mask, R, t;
        E = findEssentialMat(pts_curr, pts_prev, focal, pp, RANSAC, 0.999, 1.0, mask);
        recoverPose(E, pts_curr, pts_prev, R, t, focal, pp, mask);

        // 累加轨迹
        t_global = t_global + 1.0 * (R_global * t);
        R_global = R * R_global;

        // ==========================================
        // 💾 保存位姿到文件 (重点修改在这里)
        // ==========================================
        // 1. 把 OpenCV 的 R 转换成 Eigen 的旋转矩阵，再转为四元数
        Eigen::Matrix3d R_eigen;
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
                R_eigen(r, c) = R_global.at<double>(r, c);
        
        Eigen::Quaterniond q(R_eigen);

        // 2. 写入文件：时间戳 tx ty tz qx qy qz qw
        pose_file << std::fixed << std::setprecision(6) << frames[i].timestamp << " "
                  << t_global.at<double>(0) << " " << t_global.at<double>(1) << " " << t_global.at<double>(2) << " "
                  << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;

        // 绘制轨迹
        int draw_x = int(t_global.at<double>(0) * 5) + 300; 
        int draw_y = int(t_global.at<double>(2) * 5) + 300; 
        if (draw_x >= 0 && draw_x < 600 && draw_y >= 0 && draw_y < 600) {
            circle(trajectory, Point(draw_x, draw_y), 1, CV_RGB(0, 255, 0), FILLED);
        }

        imshow("TUM Dataset View", img_curr);
        imshow("Robot Trajectory", trajectory);
        waitKey(1); // 提速，方便快速跑完序列

        // 传球
        img_prev = img_curr.clone();
        kp_prev = kp_curr;
        desc_prev = desc_curr.clone();
    }

    pose_file.close();
    cout << "🏁 序列播放完毕，位姿已保存至 results/camera_poses.txt！" << endl;
    waitKey(0);
    return 0;
}
