# 1. 指定基础镜像：直接使用官方的 ROS Noetic 完整版环境
FROM osrf/ros:noetic-desktop-full

# 2. 标明维护者信息（换成你自己的名字）
LABEL maintainer="zyj <910264028@qq.com>"

# 3. 避免安装过程中的交互式弹窗卡死
ENV DEBIAN_FRONTEND=noninteractive

# 4. 更新软件源并安装必要的依赖（包含我们刚用的 evo 评测工具）
RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-noetic-cv-bridge \
    ros-noetic-tf2-eigen \
    ros-noetic-pcl-conversions \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install evo --upgrade --no-binary evo

# 5. 在容器内创建工业园区（工作空间）
RUN mkdir -p /root/slam_ws/src/my_slam
WORKDIR /root/slam_ws

# 6. 把你电脑上写好的代码，全部拷贝进容器的 my_slam 文件夹里
COPY . /root/slam_ws/src/my_slam/

# 7. 在容器内进行终极编译！
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# 8. 添加入口脚本：容器启动时自动 source 环境变量
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /root/slam_ws/devel/setup.bash" >> ~/.bashrc

# 9. 默认启动命令：一键拉起建图全家桶！
CMD ["/bin/bash", "-c", "source /root/slam_ws/devel/setup.bash && roslaunch my_slam slam_run.launch"]
