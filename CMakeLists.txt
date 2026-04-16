cmake_minimum_required(VERSION 3.0.2)
project(my_slam)
# 开启最高级别编译优化，并引入 OpenMP 多线程支持
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fopenmp")
find_package(OpenMP REQUIRED)
# 1. 寻找 ROS 依赖 (新增了 cv_bridge, message_filters 和 tf2 家族)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  pcl_conversions
  cv_bridge
  message_filters
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
)

# 2. 寻找第三方依赖
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

# 3. 声明 ROS 包配置
catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs nav_msgs geometry_msgs pcl_conversions cv_bridge message_filters tf2 tf2_ros tf2_eigen tf2_geometry_msgs
)

# 4. 包含头文件目录
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# 5. 生成稠密建图节点可执行文件
add_executable(pcl_mapping src/pcl_mapping.cpp)
target_link_libraries(pcl_mapping ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(vo_sequence src/vo_sequence.cpp)
target_link_libraries(vo_sequence ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})






