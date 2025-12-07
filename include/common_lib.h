/* This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef COMMON_LIB_H
#define COMMON_LIB_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <cstdarg>
#include <cstdio>
#include <deque>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iostream>
#include <nav_msgs/msg/odometry.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sophus/se3.hpp>
#include <string>
#include <tf2_eigen/tf2_eigen.hpp>
#include <vector>

#include "utils/color.h"
#include "utils/so3_math.h"
#include "utils/types.h"

using namespace std;
using namespace Eigen;
using namespace Sophus;

// ==================================================================
// =================== 日志输出宏 (适配 ROS 2) ===================
// ==================================================================

#define ANSI_COLOR_RED "\x1b[31m"
#define ANSI_COLOR_YELLOW "\x1b[33m"
#define ANSI_COLOR_BLUE "\x1b[34m"
#define ANSI_COLOR_RESET "\x1b[0m"

inline std::string format_string_for_log(const char* format, ...) {
  char buffer[2048];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  return std::string(buffer);
}

#define LOG_INFO(message)                                                   \
  std::cout << "[" << __FILE__ << ":" << __LINE__ << "] [INFO] " << message \
            << std::endl;

#define LOG_WARN(message)                                                      \
  std::cout << "[" << __FILE__ << ":" << __LINE__ << "] " << ANSI_COLOR_YELLOW \
            << "[WARN] " << message << ANSI_COLOR_RESET << std::endl;

#define LOG_ERROR(message)                                                  \
  std::cerr << "[" << __FILE__ << ":" << __LINE__ << "] " << ANSI_COLOR_RED \
            << "[ERROR] " << message << ANSI_COLOR_RESET << std::endl;

#define LOG_INFO_F(format, ...)                                  \
  std::cout << "[" << __FILE__ << ":" << __LINE__ << "] [INFO] " \
            << format_string_for_log(format, ##__VA_ARGS__) << std::endl;

#define LOG_WARN_F(format, ...)                                                \
  std::cout << "[" << __FILE__ << ":" << __LINE__ << "] " << ANSI_COLOR_YELLOW \
            << "[WARN] " << format_string_for_log(format, ##__VA_ARGS__)       \
            << ANSI_COLOR_RESET << std::endl;

#define LOG_ERROR_F(format, ...)                                            \
  std::cerr << "[" << __FILE__ << ":" << __LINE__ << "] " << ANSI_COLOR_RED \
            << "[ERROR] " << format_string_for_log(format, ##__VA_ARGS__)   \
            << ANSI_COLOR_RESET << std::endl;

// ==================================================================

#define G_m_s2 (9.81)
#define DIM_STATE (19)
#define INIT_COV (0.01)
#define SIZE_LARGE (500)
#define SIZE_SMALL (100)
#define VEC_FROM_ARRAY(v) v[0], v[1], v[2]
#define MAT_FROM_ARRAY(v) v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8]
#define DEBUG_FILE_DIR(name) (string(string(ROOT_DIR) + "Log/" + name))

enum LID_TYPE {
  AVIA = 1,
  VELO16 = 2,
  OUST64 = 3,
  L515 = 4,
  XT32 = 5,
  PANDAR128 = 6,
  ROBOSENSE = 7
};
enum SLAM_MODE { ONLY_LO = 0, ONLY_LIO = 1, LIVO = 2 };
enum EKF_STATE { WAIT = 0, VIO = 1, LIO = 2, LO = 3 };

struct MeasureGroup {
  double vio_time;
  double lio_time;
  deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu;
  cv::Mat img;
  MeasureGroup() {
    vio_time = 0.0;
    lio_time = 0.0;
  };
};

struct LidarMeasureGroup {
  double lidar_frame_beg_time;
  double lidar_frame_end_time;
  double last_lio_update_time;
  PointCloudXYZI::Ptr lidar;
  PointCloudXYZI::Ptr pcl_proc_cur;
  PointCloudXYZI::Ptr pcl_proc_next;
  deque<struct MeasureGroup> measures;
  EKF_STATE lio_vio_flg;
  int lidar_scan_index_now;

  LidarMeasureGroup() {
    lidar_frame_beg_time = -0.0;
    lidar_frame_end_time = 0.0;
    last_lio_update_time = -1.0;
    lio_vio_flg = WAIT;
    this->lidar.reset(new PointCloudXYZI());
    this->pcl_proc_cur.reset(new PointCloudXYZI());
    this->pcl_proc_next.reset(new PointCloudXYZI());
    this->measures.clear();
    lidar_scan_index_now = 0;
    last_lio_update_time = -1.0;
  };
};

inline double get_time_sec(const builtin_interfaces::msg::Time& time) {
  return rclcpp::Time(time).seconds();
}

inline rclcpp::Time get_ros_time(double timestamp) {
  int32_t sec = std::floor(timestamp);
  auto nanosec_d = (timestamp - std::floor(timestamp)) * 1e9;
  uint32_t nanosec = static_cast<uint32_t>(nanosec_d);
  return rclcpp::Time(sec, nanosec);
}

typedef struct pointWithVar {
  Eigen::Vector3d point_b;
  Eigen::Vector3d point_i;
  Eigen::Vector3d point_w;
  Eigen::Matrix3d var_nostate;
  Eigen::Matrix3d body_var;
  Eigen::Matrix3d var;
  Eigen::Matrix3d point_crossmat;
  Eigen::Vector3d normal;
  float intensity;
  pointWithVar() {
    var_nostate = Eigen::Matrix3d::Zero();
    var = Eigen::Matrix3d::Zero();
    body_var = Eigen::Matrix3d::Zero();
    point_crossmat = Eigen::Matrix3d::Zero();
    point_b = Eigen::Vector3d::Zero();
    point_i = Eigen::Vector3d::Zero();
    point_w = Eigen::Vector3d::Zero();
    normal = Eigen::Vector3d::Zero();
  };
} pointWithVar;

struct StatesGroup {
  StatesGroup() {
    this->rot_end = M3D::Identity();
    this->pos_end = V3D::Zero();
    this->vel_end = V3D::Zero();
    this->bias_g = V3D::Zero();
    this->bias_a = V3D::Zero();
    this->gravity = V3D::Zero();
    this->inv_expo_time = 1.0;
    this->cov =
        Eigen::Matrix<double, DIM_STATE, DIM_STATE>::Identity() * INIT_COV;
    this->cov(6, 6) = 0.00001;
    this->cov.block<9, 9>(10, 10) =
        Eigen::Matrix<double, 9, 9>::Identity() * 0.00001;
  };
  StatesGroup(const StatesGroup& b) {
    this->rot_end = b.rot_end;
    this->pos_end = b.pos_end;
    this->vel_end = b.vel_end;
    this->bias_g = b.bias_g;
    this->bias_a = b.bias_a;
    this->gravity = b.gravity;
    this->inv_expo_time = b.inv_expo_time;
    this->cov = b.cov;
  };
  StatesGroup& operator=(const StatesGroup& b) {
    this->rot_end = b.rot_end;
    this->pos_end = b.pos_end;
    this->vel_end = b.vel_end;
    this->bias_g = b.bias_g;
    this->bias_a = b.bias_a;
    this->gravity = b.gravity;
    this->inv_expo_time = b.inv_expo_time;
    this->cov = b.cov;
    return *this;
  };
  StatesGroup operator+(const Eigen::Matrix<double, DIM_STATE, 1>& state_add) {
    StatesGroup a;
    a.rot_end =
        this->rot_end * Exp(state_add(0, 0), state_add(1, 0), state_add(2, 0));
    a.pos_end = this->pos_end + state_add.block<3, 1>(3, 0);
    a.inv_expo_time = this->inv_expo_time + state_add(6, 0);
    a.vel_end = this->vel_end + state_add.block<3, 1>(7, 0);
    a.bias_g = this->bias_g + state_add.block<3, 1>(10, 0);
    a.bias_a = this->bias_a + state_add.block<3, 1>(13, 0);
    a.gravity = this->gravity + state_add.block<3, 1>(16, 0);
    a.cov = this->cov;
    return a;
  };
  StatesGroup& operator+=(
      const Eigen::Matrix<double, DIM_STATE, 1>& state_add) {
    this->rot_end =
        this->rot_end * Exp(state_add(0, 0), state_add(1, 0), state_add(2, 0));
    this->pos_end += state_add.block<3, 1>(3, 0);
    this->inv_expo_time += state_add(6, 0);
    this->vel_end += state_add.block<3, 1>(7, 0);
    this->bias_g += state_add.block<3, 1>(10, 0);
    this->bias_a += state_add.block<3, 1>(13, 0);
    this->gravity += state_add.block<3, 1>(16, 0);
    return *this;
  };
  Eigen::Matrix<double, DIM_STATE, 1> operator-(const StatesGroup& b) {
    Eigen::Matrix<double, DIM_STATE, 1> a;
    M3D rotd(b.rot_end.transpose() * this->rot_end);
    a.block<3, 1>(0, 0) = Log(rotd);
    a.block<3, 1>(3, 0) = this->pos_end - b.pos_end;
    a(6, 0) = this->inv_expo_time - b.inv_expo_time;
    a.block<3, 1>(7, 0) = this->vel_end - b.vel_end;
    a.block<3, 1>(10, 0) = this->bias_g - b.bias_g;
    a.block<3, 1>(13, 0) = this->bias_a - b.bias_a;
    a.block<3, 1>(16, 0) = this->gravity - b.gravity;
    return a;
  };
  void resetpose() {
    this->rot_end = M3D::Identity();
    this->pos_end = V3D::Zero();
    this->vel_end = V3D::Zero();
  }
  M3D rot_end;
  V3D pos_end;
  V3D vel_end;
  double inv_expo_time;
  V3D bias_g;
  V3D bias_a;
  V3D gravity;
  Eigen::Matrix<double, DIM_STATE, DIM_STATE> cov;
};

template <typename T>
auto set_pose6d(const double t, const Eigen::Matrix<T, 3, 1>& a,
                const Eigen::Matrix<T, 3, 1>& g,
                const Eigen::Matrix<T, 3, 1>& v,
                const Eigen::Matrix<T, 3, 1>& p,
                const Eigen::Matrix<T, 3, 3>& R) {
  Pose6D rot_kp;
  rot_kp.offset_time = t;
  for (size_t i = 0; i < 3; ++i) {
    rot_kp.acc[i] = a(i);
    rot_kp.gyr[i] = g(i);
    rot_kp.vel[i] = v(i);
    rot_kp.pos[i] = p(i);
    for (size_t j = 0; j < 3; ++j) rot_kp.rot[i * 3 + j] = R(i, j);
  }
  return rot_kp;
}

#endif