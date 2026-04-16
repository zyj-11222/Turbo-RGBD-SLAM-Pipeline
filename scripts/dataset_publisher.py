#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import tf2_ros
import geometry_msgs.msg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os

def load_file_list(filename):
    """辅助函数：读取 txt 字典"""
    data = {}
    if not os.path.exists(filename):
        return data
    with open(filename, 'r') as f:
        for line in f:
            if line.startswith('#') or not line.strip():
                continue
            parts = line.split()
            data[float(parts[0])] = parts[1]
    return data

def main():
    rospy.init_node('dataset_publisher')
    
    # 1. 从参数服务器动态获取路径，和 C++ 节点保持完美一致！
    dataset_dir = rospy.get_param('/dataset_dir', '/home/zyj/slam_ws/src/my_slam/sequence/')
    pose_file_path = rospy.get_param('/pose_file_path', os.path.join(dataset_dir, 'groundtruth.txt'))
    
    # 2. 初始化发布者和 TF 广播员
    rgb_pub = rospy.Publisher('/camera/rgb/image_raw', Image, queue_size=10)
    depth_pub = rospy.Publisher('/camera/depth/image_raw', Image, queue_size=10)
    tf_broadcaster = tf2_ros.TransformBroadcaster()
    bridge = CvBridge()
    
    rospy.loginfo("💿 正在加载 TUM 数据集并准备广播...")
    rgb_dict = load_file_list(os.path.join(dataset_dir, 'rgb.txt'))
    depth_dict = load_file_list(os.path.join(dataset_dir, 'depth.txt'))
    
    rgb_times = sorted(rgb_dict.keys())
    depth_times = sorted(depth_dict.keys())
    
    if not rgb_times or not depth_times:
        rospy.logerr("🚨 未找到图像数据，请检查路径！")
        return
        
    rate = rospy.Rate(30) # 模拟 30Hz 的真实相机帧率
    
    with open(pose_file_path, 'r') as f:
        for line in f:
            if rospy.is_shutdown():
                break
            if line.startswith('#') or not line.strip():
                continue
                
            parts = line.split()
            t_pose = float(parts[0])
            tx, ty, tz = float(parts[1]), float(parts[2]), float(parts[3])
            qx, qy, qz, qw = float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])
            
            # 找到时间最接近的图像
            closest_rgb_t = min(rgb_times, key=lambda x: abs(x - t_pose))
            closest_depth_t = min(depth_times, key=lambda x: abs(x - t_pose))
            
            if abs(closest_rgb_t - t_pose) > 0.05 or abs(closest_depth_t - t_pose) > 0.05:
                continue # 时间差太大，丢弃这一帧
                
            rgb_img = cv2.imread(os.path.join(dataset_dir, rgb_dict[closest_rgb_t]))
            depth_img = cv2.imread(os.path.join(dataset_dir, depth_dict[closest_depth_t]), cv2.IMREAD_UNCHANGED)
            
            if rgb_img is None or depth_img is None:
                continue
                
            # 💡 核心技巧：强制将所有消息打上同一时刻的系统时间，确保 C++ 端的 message_filters 能完美同步！
            current_time = rospy.Time.now()
            
            # 广播 TF 树 (从 map 到 camera_link)
            tfs = geometry_msgs.msg.TransformStamped()
            tfs.header.stamp = current_time
            tfs.header.frame_id = "map"
            tfs.child_frame_id = "camera_link"
            tfs.transform.translation.x = tx
            tfs.transform.translation.y = ty
            tfs.transform.translation.z = tz
            tfs.transform.rotation.x = qx
            tfs.transform.rotation.y = qy
            tfs.transform.rotation.z = qz
            tfs.transform.rotation.w = qw
            tf_broadcaster.sendTransform(tfs)
            
            # 发送 ROS 图像话题
            rgb_msg = bridge.cv2_to_imgmsg(rgb_img, "bgr8")
            rgb_msg.header.stamp = current_time
            rgb_msg.header.frame_id = "camera_link"
            rgb_pub.publish(rgb_msg)
            
            depth_msg = bridge.cv2_to_imgmsg(depth_img, "16UC1")
            depth_msg.header.stamp = current_time
            depth_msg.header.frame_id = "camera_link"
            depth_pub.publish(depth_msg)
            
            rospy.loginfo_once("✅ 数据流已成功启动，正在向 C++ 节点发送数据...")
            rate.sleep()
            
    rospy.loginfo("🎉 数据集播放完毕！")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
