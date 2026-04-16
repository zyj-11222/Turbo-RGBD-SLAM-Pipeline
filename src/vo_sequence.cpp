#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <iomanip>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

// ==========================================
// 🚀 ROS 1 核心头文件
// ==========================================
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;
using namespace cv;

// 定义一个结构体来同时存储时间戳和文件名
struct FrameInfo {
    string timestamp;
    string filename;
};

int main(int argc, char** argv) {
    // 1. 初始化 ROS 1
    ros::init(argc, argv, "vo_node");
    ros::NodeHandle nh;
    
    // 创建一个发布者，发布的话题名叫 "camera_path"
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("camera_path", 10, true);

    // 定义我们要发布的路径消息
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "map"; // 极其重要：告诉 RViz 这个轨迹在哪个坐标系下

    string dataset_dir = "/home/zyj/slam_ws/src/my_slam/sequence/";
    string list_file = dataset_dir + "rgb.txt";

    // 2. 初始化保存文件逻辑
    std::ofstream pose_file("/home/zyj/slam_ws/src/my_slam/sequence/camera_poses.txt");
    if (!pose_file.is_open()) {
        std::cerr << "🚨 无法创建位姿文件！请检查是否已经创建了 results 文件夹。" << std::endl;
        return -1;
    }

    // 3. 读取 rgb.txt 图片清单（包含时间戳）
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

    // 4. 初始化全局位姿
    Mat R_global = Mat::eye(3, 3, CV_64F);
    Mat t_global = Mat::zeros(3, 1, CV_64F);

    Ptr<FeatureDetector> detector = ORB::create(500);
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

    Mat img_prev, desc_prev;
    vector<KeyPoint> kp_prev;

    // TUM 数据集相机内参
    double focal = 517.3; 
    Point2d pp(318.6, 255.3);

    // 窗口排版 (只保留相机视图，轨迹交由 RViz 显示)
    namedWindow("TUM Dataset View", WINDOW_NORMAL);
    resizeWindow("TUM Dataset View", 640, 480);

    // 处理第一张图
    img_prev = imread(dataset_dir + frames[0].filename, IMREAD_GRAYSCALE);
    if (img_prev.empty()) { cout << "🚨 读不到第一张图！" << endl; return -1; }
    detector->detectAndCompute(img_prev, noArray(), kp_prev, desc_prev);

    // 5. 开始接力循环 (加入 ros::ok() 以便支持 Ctrl+C 中断)
    for (size_t i = 1; i < frames.size() && ros::ok(); i++) {
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

        // 把 OpenCV 的 R 转换成 Eigen 的旋转矩阵，再转为四元数
        Eigen::Matrix3d R_eigen;
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
                R_eigen(r, c) = R_global.at<double>(r, c);
        
        Eigen::Quaterniond q(R_eigen);

        // 写入位姿文件
        pose_file << std::fixed << std::setprecision(6) << frames[i].timestamp << " "
                  << t_global.at<double>(0) << " " << t_global.at<double>(1) << " " << t_global.at<double>(2) << " "
                  << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;

        // 显示相机实时画面
        imshow("TUM Dataset View", img_curr);
        waitKey(1); // 提速，方便快速跑完序列

        // ==========================================
        // 🚀 ROS 1 发布轨迹逻辑
        // ==========================================
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now(); // 打上当前的时间戳
        pose.header.frame_id = "map";    // 统一参考系

        // 填入平移 (注意 OpenCV 矩阵元素的读取)
        pose.pose.position.x = t_global.at<double>(0);
        pose.pose.position.y = t_global.at<double>(1);
        pose.pose.position.z = t_global.at<double>(2);

        // 填入旋转 (四元数)
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();

        // 把当前点塞进路径里，并发布出去！
        path_msg.poses.push_back(pose);
        path_pub.publish(path_msg);

        // 让 ROS 1 处理一下底层通讯回调
        ros::spinOnce();

        // 传球：当前帧变成下一轮的上一帧
        img_prev = img_curr.clone();
        kp_prev = kp_curr;
        desc_prev = desc_curr.clone();
    }

    pose_file.close();
    cout << "🏁 序列播放完毕，位姿已保存至 results/camera_poses.txt！" << endl;
    ros::spin();
    return 0;
}
