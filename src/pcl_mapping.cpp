#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <std_srvs/Empty.h> // ROS 标准空服务

// PCL 核心库
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>  // PCL 文件保存库
#include <omp.h>    // OpenMP 多线程库
#include <chrono>   // C++ 高精度计时器
using namespace std;

class DenseMapping {
private:
    ros::NodeHandle nh_;
    ros::Publisher map_pub_;
    ros::ServiceServer save_srv_; // 保存地图的服务
    
    // TF 监听器：实时获取相机位姿
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // 消息同步订阅者：确保 RGB 和 Depth 对齐
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub_;
    message_filters::Subscriber<sensor_msgs::Image> depth_sub_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Sync;
    boost::shared_ptr<Sync> sync_;

    // 核心参数
    double cx_, cy_, fx_, fy_, depth_scale_;
    double voxel_size_, z_min_, z_max_;
    string world_frame_, camera_frame_;

    // PCL 地图容器与滤波器
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_map_;
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter_;

public:
    DenseMapping() : tf_listener_(tf_buffer_) {
        // 1. 加载参数服务器配置
        nh_.param<double>("/camera/cx", cx_, 318.6);
        nh_.param<double>("/camera/cy", cy_, 255.3);
        nh_.param<double>("/camera/fx", fx_, 517.3);
        nh_.param<double>("/camera/fy", fy_, 516.5);
        nh_.param<double>("/camera/depth_scale", depth_scale_, 5000.0);
        nh_.param<double>("/mapping/voxel_size", voxel_size_, 0.05);
        nh_.param<double>("/mapping/z_min", z_min_, 0.2);
        nh_.param<double>("/mapping/z_max", z_max_, 3.0);
        nh_.param<string>("/mapping/world_frame", world_frame_, "map");
        nh_.param<string>("/mapping/camera_frame", camera_frame_, "camera_link");

        // 2. 初始化点云容器与滤波器
        global_map_.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
        voxel_filter_.setLeafSize(voxel_size_, voxel_size_, voxel_size_);

        // 3. 注册发布者与服务
        map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("point_cloud_map", 1);
        save_srv_ = nh_.advertiseService("/save_map", &DenseMapping::saveMapCallback, this);

        // 4. 初始化同步订阅者 (队列长度设为10)
        rgb_sub_.subscribe(nh_, "/camera/rgb/image_raw", 10);
        depth_sub_.subscribe(nh_, "/camera/depth/image_raw", 10);
        sync_.reset(new Sync(SyncPolicy(10), rgb_sub_, depth_sub_));
        
        // 绑定回调函数
        sync_->registerCallback(boost::bind(&DenseMapping::imageCallback, this, _1, _2));

        ROS_INFO("🚀 实时建图节点已启动！");
        ROS_INFO("📡 正在监听话题: /camera/rgb/image_raw 和 /camera/depth/image_raw");
        ROS_INFO("💾 随时可以在新终端输入 rosservice call /save_map 保存地图！");
    }

    // ==========================================
    // 核心回调函数：处理对齐后的 RGB 和 Depth 图像
    // ==========================================
    void imageCallback(const sensor_msgs::ImageConstPtr& rgb_msg, const sensor_msgs::ImageConstPtr& depth_msg) {
        // A. 通过 TF 获取当前时刻相机的位姿 T_wc
        geometry_msgs::TransformStamped transform_stamped;
        try {
            transform_stamped = tf_buffer_.lookupTransform(world_frame_, camera_frame_, rgb_msg->header.stamp, ros::Duration(0.1));
        } catch (tf2::TransformException &ex) {
            ROS_WARN("TF 查找失败，跳过此帧: %s", ex.what());
            return;
        }

        Eigen::Isometry3d T = tf2::transformToEigen(transform_stamped);

        // B. 转换 ROS 图像为 OpenCV 格式
        cv_bridge::CvImageConstPtr cv_ptr_rgb, cv_ptr_depth;
        try {
            cv_ptr_rgb = cv_bridge::toCvShare(rgb_msg, "bgr8");
            cv_ptr_depth = cv_bridge::toCvShare(depth_msg, "16UC1");
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge 异常: %s", e.what());
            return;
        }

        // C. 生成局部点云 (🚀 里程碑 2：OpenMP 多线程加速)
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        
        auto start_time = std::chrono::high_resolution_clock::now();

        #pragma omp parallel
        {
            pcl::PointCloud<pcl::PointXYZRGB> local_cloud;

            #pragma omp for nowait
            for (int v = 0; v < cv_ptr_rgb->image.rows; v += 1) { 
                for (int u = 0; u < cv_ptr_rgb->image.cols; u += 1) {
                    unsigned short d = cv_ptr_depth->image.at<unsigned short>(v, u);
                    if (d == 0) continue;
                    
                    double z = double(d) / depth_scale_;
                    if (z < z_min_ || z > z_max_) continue;

                    pcl::PointXYZRGB p;
                    p.z = z;
                    p.x = (u - cx_) * z / fx_;
                    p.y = (v - cy_) * z / fy_;
                    p.b = cv_ptr_rgb->image.at<cv::Vec3b>(v, u)[0];
                    p.g = cv_ptr_rgb->image.at<cv::Vec3b>(v, u)[1];
                    p.r = cv_ptr_rgb->image.at<cv::Vec3b>(v, u)[2];
                    
                    local_cloud.push_back(p);
                }
            }

            #pragma omp critical
            {
                *current_cloud += local_cloud;
            }
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
        ROS_INFO("⚡ 单帧全分辨率点云生成耗时: %ld ms, 共生成 %d 个点", duration, (int)current_cloud->size());

        // D. 把局部点云变换到世界坐标系
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::transformPointCloud(*current_cloud, *transformed_cloud, T.matrix());

        // E. 拼接到全局地图
        *global_map_ += *transformed_cloud;

        // F. 定期进行体素滤波并发布给 RViz
        static int count = 0;
        if (++count % 5 == 0) { 
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_map(new pcl::PointCloud<pcl::PointXYZRGB>());
            voxel_filter_.setInputCloud(global_map_);
            voxel_filter_.filter(*filtered_map);
            
            *global_map_ = *filtered_map;

            sensor_msgs::PointCloud2 ros_out;
            pcl::toROSMsg(*global_map_, ros_out);
            ros_out.header.stamp = ros::Time::now();
            ros_out.header.frame_id = world_frame_;
            map_pub_.publish(ros_out);
        }
    }

    // ==========================================
    // 服务回调函数：一键保存 PCD 文件
    // ==========================================
    bool saveMapCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
        if (global_map_->empty()) {
            ROS_WARN("🚨 地图为空，无法保存！请先接收数据并建图。");
            return false;
        }
        
        string save_path = "/home/zyj/slam_ws/my_awesome_desk.pcd";
        ROS_INFO("⏳ 正在保存地图到: %s", save_path.c_str());
        
        // 保存为二进制 PCD 格式 (体积更小，读取更快)
        pcl::io::savePCDFileBinary(save_path, *global_map_);
        
        ROS_INFO("✅ 地图保存成功！共计包含 %d 个 3D 点。", (int)global_map_->size());
        return true;
    }
};

// ==========================================
// 极简 Main 函数
// ==========================================
int main(int argc, char** argv) {
    // 解决 ROS_INFO 中文乱码问题
    setlocale(LC_ALL, "");
    
    ros::init(argc, argv, "dense_mapping_node");
    DenseMapping mapper;
    
    // ros::spin() 会让节点进入死循环，持续等待图像数据到来或服务被调用
    ros::spin(); 
    
    return 0;
}
