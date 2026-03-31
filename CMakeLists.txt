cmake_minimum_required(VERSION 3.10)
project(Mini_SLAM_System)

set(CMAKE_CXX_STANDARD 17)

# 1. 寻找核心依赖库
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io kdtree registration filters visualization)

# 2. 包含头文件路径
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 3. 编译视觉模块 (OpenCV + Ceres)
add_executable(vo_sequence src/vo_sequence.cpp)
target_link_libraries(vo_sequence ${OpenCV_LIBS})

add_executable(ceres_ba src/ceres_ba.cpp)
target_link_libraries(ceres_ba ${OpenCV_LIBS} Ceres::ceres)

# 4. 编译激光模块 (PCL)
add_executable(icp_test src/icp_test.cpp)
target_link_libraries(icp_test ${PCL_LIBRARIES})

add_executable(pcl_visualize src/pcl_visualize.cpp)
target_link_libraries(pcl_visualize ${PCL_LIBRARIES})

add_executable(pcl_mapping src/pcl_mapping.cpp)
target_link_libraries(pcl_mapping ${PCL_LIBRARIES})
