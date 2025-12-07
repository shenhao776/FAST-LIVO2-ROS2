cmake_minimum_required(VERSION 3.8)
project(map_updater)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 优化选项
add_compile_options(-O3)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(pcl_ros REQUIRED)

# 【新增】 查找 vikit 依赖包
# 确保先在 package.xml 中添加了 <depend>vikit_ros</depend>，否则这里会报错找不到
find_package(vikit_common REQUIRED)
find_package(vikit_ros REQUIRED)

# 第三方库
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io kdtree search filters registration segmentation features)
find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)
find_package(OpenMP QUIET)

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

# 包含路径配置
include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  # 如果 vikit 是标准 ament 包，ament_target_dependencies 会自动处理包含路径
  # 但为了保险，保留变量引用（如果它们定义了的话）
  ${vikit_common_INCLUDE_DIRS}
  ${vikit_ros_INCLUDE_DIRS}
)

# 库文件
add_library(map_updater_lib SHARED
  src/preprocess.cpp
  src/IMU_Processing.cpp
  src/voxel_map.cpp
  src/frame.cpp
  src/visual_point.cpp
  src/vio.cpp
  src/LIVMapper.cpp
)

# 链接库
target_link_libraries(map_updater_lib
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Sophus_LIBRARIES}
  # 如果 vikit 库导出了变量则链接
  ${vikit_common_LIBRARIES}
  ${vikit_ros_LIBRARIES}
)

if(OpenMP_CXX_FOUND)
  target_link_libraries(map_updater_lib OpenMP::OpenMP_CXX)
endif()

# 使用 ament 自动处理依赖的头文件和库链接
ament_target_dependencies(map_updater_lib
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  pcl_conversions
  livox_ros_driver2
  cv_bridge
  image_transport
  pcl_ros
  # 【关键】添加 vikit 依赖
  vikit_common
  vikit_ros
)

# 可执行文件
add_executable(map_updater_node src/main.cpp)
target_link_libraries(map_updater_node map_updater_lib)

# 节点同样需要依赖
ament_target_dependencies(map_updater_node 
  rclcpp
  vikit_common 
  vikit_ros
)

# 安装规则
install(TARGETS map_updater_lib map_updater_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch rviz_cfg
  DESTINATION share/${PROJECT_NAME}
)

# 导出依赖关系
ament_export_dependencies(
  rclcpp
  vikit_common
  vikit_ros
)

ament_package()