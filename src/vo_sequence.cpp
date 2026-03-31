#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main() {
    string dataset_dir = "sequence/";
    string list_file = dataset_dir + "rgb.txt";

    // 1. 读取 rgb.txt 图片清单
    ifstream fin(list_file);
    if (!fin) {
        cout << "🚨 找不到 rgb.txt！请检查 sequence 文件夹里有没有放对位置！" << endl;
        return -1;
    }

    vector<string> image_filenames;
    string line;
    while (getline(fin, line)) {
        if (line.empty() || line[0] == '#') continue; // 跳过开头的注释行
        
        istringstream iss(line);
        string timestamp, filename;
        iss >> timestamp >> filename; // 读取时间戳和后面的相对路径(如 rgb/xxx.png)
        image_filenames.push_back(filename);
    }
    cout << "✅ 成功读取图片清单，共 " << image_filenames.size() << " 张图片。" << endl;

    // 2. 初始化全局位姿和画布
    Mat R_global = Mat::eye(3, 3, CV_64F);
    Mat t_global = Mat::zeros(3, 1, CV_64F);
    Mat trajectory = Mat::zeros(600, 600, CV_8UC3); // 600x600 纯黑画布

    Ptr<FeatureDetector> detector = ORB::create(500);
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

    Mat img_prev, desc_prev;
    vector<KeyPoint> kp_prev;

    // 3. TUM 数据集的相机内参 (fr1 相机的标准参数)
    double focal = 517.3; 
    Point2d pp(318.6, 255.3);
    // ==========================================
    // 📺 优化窗口显示：强行排版
    // ==========================================
    // 1. 设置相机画面的窗口
    namedWindow("TUM Dataset View", WINDOW_NORMAL);
    resizeWindow("TUM Dataset View", 640, 480); // TUM 默认分辨率
    moveWindow("TUM Dataset View", 50, 100);    // 放在屏幕左上角 (X=50, Y=100)

    // 2. 设置轨迹画面的窗口
    namedWindow("Robot Trajectory", WINDOW_NORMAL);
    resizeWindow("Robot Trajectory", 600, 600);
    moveWindow("Robot Trajectory", 50 + 640 + 20, 100); // 紧挨着相机画面放在右边
    // 读取第一张图作为老祖宗
    img_prev = imread(dataset_dir + image_filenames[0], IMREAD_GRAYSCALE);
    if (img_prev.empty()) { cout << "🚨 读不到图片！" << endl; return -1; }
    detector->detectAndCompute(img_prev, noArray(), kp_prev, desc_prev);

    // 4. 开始极其燃的接力循环！
    for (size_t i = 1; i < image_filenames.size(); i++) {
        Mat img_curr = imread(dataset_dir + image_filenames[i], IMREAD_GRAYSCALE);
        if (img_curr.empty()) continue;

        vector<KeyPoint> kp_curr; Mat desc_curr;
        detector->detectAndCompute(img_curr, noArray(), kp_curr, desc_curr);

        // 匹配与筛选
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

        // 踢掉特征点太少的帧，防止矩阵计算崩溃
        if (pts_prev.size() < 12) continue;

        // 计算相对位姿
        Mat E, mask, R, t;
        E = findEssentialMat(pts_curr, pts_prev, focal, pp, RANSAC, 0.999, 1.0, mask);
        recoverPose(E, pts_curr, pts_prev, R, t, focal, pp, mask);

        // 累加轨迹！(单目假定每次步长为 1.0)
        t_global = t_global + 1.0 * (R_global * t);
        R_global = R * R_global;

        // 在画布上画出当前位置 
        // 放大 20 倍，并且把起始点移到画布正中间 (300, 300)
        int draw_x = int(t_global.at<double>(0) * 5) + 300; 
        int draw_y = int(t_global.at<double>(2) * 5) + 300; 
        
        // 防止画到屏幕外面去导致段错误
        if (draw_x >= 0 && draw_x < 600 && draw_y >= 0 && draw_y < 600) {
            circle(trajectory, Point(draw_x, draw_y), 1, CV_RGB(0, 255, 0), FILLED); // 绿色的轨迹
        }

        // 显示动画
        imshow("TUM Dataset View", img_curr);
        imshow("Robot Trajectory", trajectory);
        waitKey(10); // 10ms 延迟，相当于 100fps 播放

        // 传球：当前帧变成下一轮的上一帧
        img_prev = img_curr.clone();
        kp_prev = kp_curr;
        desc_prev = desc_curr.clone();
    }

    cout << "🏁 序列播放完毕！按任意键退出..." << endl;
    waitKey(0);
    return 0;
}
