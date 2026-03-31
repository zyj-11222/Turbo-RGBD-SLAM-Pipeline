#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace std;
using namespace cv;

int main(int argc, char** argv) {
    Mat img_1 = imread("1.png", IMREAD_COLOR);
    Mat img_2 = imread("2.png", IMREAD_COLOR);
    if (img_1.empty() || img_2.empty()) { cout << "找不到图片！" << endl; return -1; }

    // 1. ORB 特征提取与匹配
    vector<KeyPoint> kp1, kp2;
    Mat desc1, desc2;
    Ptr<ORB> orb = ORB::create(500, 1.2f, 8, 31, 0, 2, ORB::HARRIS_SCORE, 31, 20);
    orb->detectAndCompute(img_1, Mat(), kp1, desc1);
    orb->detectAndCompute(img_2, Mat(), kp2, desc2);

    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
    vector<DMatch> matches;
    matcher->match(desc1, desc2, matches);

    // 筛选好的匹配
    double min_dist = 1000;
    for (auto m : matches) if (m.distance < min_dist) min_dist = m.distance;
    vector<DMatch> good_matches;
    for (auto m : matches) if (m.distance <= max(2 * min_dist, 30.0)) good_matches.push_back(m);

    vector<Point2f> pts1, pts2;
    for (auto m : good_matches) {
        pts1.push_back(kp1[m.queryIdx].pt);
        pts2.push_back(kp2[m.trainIdx].pt);
    }

    // 2. 计算本质矩阵 & 恢复位姿
    double fx = img_1.cols * 1.2;
    double fy = img_1.cols * 1.2;
    double cx = img_1.cols / 2.0;
    double cy = img_1.rows / 2.0;
    Mat K = (Mat_<double>(3,3) << fx,0,cx, 0,fy,cy, 0,0,1);

    Mat mask;
    Mat E = findEssentialMat(pts1, pts2, fx, Point2d(cx,cy), RANSAC, 0.999, 1.0, mask);
    Mat R, t;
    recoverPose(E, pts1, pts2, R, t, fx, Point2d(cx,cy), mask);

    // 提取内点
    vector<Point2f> in1, in2;
    for (int i=0; i<mask.rows; i++) {
        if (mask.at<uchar>(i)) {
            in1.push_back(pts1[i]);
            in2.push_back(pts2[i]);
        }
    }

    // 3. 构造投影矩阵（正确！）
    Mat P1 = (Mat_<double>(3,4) << 1,0,0,0, 0,1,0,0, 0,0,1,0);
    Mat P2 = (Mat_<double>(3,4) <<
        R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), t.at<double>(0,0),
        R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), t.at<double>(1,0),
        R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), t.at<double>(2,0));
    P1 = K * P1;
    P2 = K * P2;

    // 4. 三角化（核心修复！）
    Mat pts4d;
    triangulatePoints(P1, P2, in1, in2, pts4d); // 输出 float 类型！

    cout << "\n内点数量: " << in1.size() << endl;
    cout << "前 5 个正确深度：" << endl;
    for (int i=0; i<min(5, pts4d.cols); i++) {
        // ✅ 必须用 float 读取！！！
        float x = pts4d.at<float>(0, i);
        float y = pts4d.at<float>(1, i);
        float z = pts4d.at<float>(2, i);
        float w = pts4d.at<float>(3, i);

        double depth = z / w;
        // 过滤反向深度
        if (depth < 0) depth = -depth;
        cout << "Point " << i << " depth: " << depth << endl;
    }

    return 0;
}
