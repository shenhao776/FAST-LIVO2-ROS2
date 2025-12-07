/* This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.

Modified by: SHEN HAO
Email: shenhao776@gmail.com

This code is a modified version of the FAST-LIVO2 framework.
Original repository: https://github.com/hku-mars/FAST-LIVO2
*/

#ifndef LIV_MAPPER_H
#define LIV_MAPPER_H

#include <cv_bridge/cv_bridge.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <vikit/camera_loader.h>

#include <atomic>
#include <chrono>
#include <deque>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <image_transport/image_transport.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <thread>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "IMU_Processing.h"
#include "preprocess.h"
#include "vio.h"

class LIVMapper : public rclcpp::Node {
 public:
  explicit LIVMapper(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~LIVMapper();

  void initializeSubscribersAndPublishers();
  void initializeComponents();
  void initializeFiles();

  // 删除了 run()，因为 ROS 2 通常在 main 中使用 spin
  // void run();

  // 定时器回调代替 run 循环
  void timer_callback();

  void gravityAlignment();
  void handleFirstFrame();
  void stateEstimationAndMapping();
  void handleVIO();
  void handleLIO();
  void savePCD();
  void processImu();

  bool sync_packages(LidarMeasureGroup& meas);
  void prop_imu_once(StatesGroup& imu_prop_state, const double dt, V3D acc_avr,
                     V3D angvel_avr);
  void imu_prop_callback();  // 修改签名适配 ROS 2 Timer

  void transformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t,
                      const PointCloudXYZI::Ptr& input_cloud,
                      PointCloudXYZI::Ptr& trans_cloud);
  void pointBodyToWorld(const PointType& pi, PointType& po);

  void RGBpointBodyToWorld(PointType const* const pi, PointType* const po);

  // 回调函数更新为 ROS 2 消息类型
  void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::UniquePtr msg);
  void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::UniquePtr msg_in);
  void imu_cbk(const sensor_msgs::msg::Imu::UniquePtr msg_in);
  void img_cbk(const sensor_msgs::msg::Image::UniquePtr msg_in);

  void publish_img_rgb(const image_transport::Publisher& pubImage,
                       VIOManagerPtr vio_manager);
  void publish_frame_world(
      const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr&
          pubLaserCloudFullRes,
      VIOManagerPtr vio_manager);
  void publish_effect_world(
      const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr&
          pubLaserCloudEffect,
      const std::vector<PointToPlane>& ptpl_list);
  void publish_odometry(
      const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr&
          pubOdomAftMapped);
  void publish_mavros(
      const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr&
          mavros_pose_publisher);
  void publish_path(
      const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath);

  void readParameters();

  template <typename T>
  void set_posestamp(T& out);
  template <typename T>
  void pointBodyToWorld(const Eigen::Matrix<T, 3, 1>& pi,
                        Eigen::Matrix<T, 3, 1>& po);
  template <typename T>
  Eigen::Matrix<T, 3, 1> pointBodyToWorld(const Eigen::Matrix<T, 3, 1>& pi);
  cv::Mat getImageFromMsg(
      const sensor_msgs::msg::Image::ConstSharedPtr& img_msg);

  void saveMapCallback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  // 信号处理 (静态方法)
  static void OnSignal(int sig);

 public:
  std::mutex mtx_buffer, mtx_buffer_imu_prop;
  std::condition_variable sig_buffer;

  SLAM_MODE slam_mode_;

  std::unordered_map<VOXEL_LOCATION, VoxelOctoTree*> voxel_map;

  string root_dir;
  string lid_topic, imu_topic, seq_name, img_topic;
  V3D extT;
  M3D extR;

  int feats_down_size = 0, max_iterations = 0;

  double res_mean_last = 0.05;
  double gyr_cov = 0, acc_cov = 0, inv_expo_cov = 0;
  double blind_rgb_points = 0.0;
  double last_timestamp_lidar = -1.0, last_timestamp_imu = -1.0,
         last_timestamp_img = -1.0;
  double filter_size_surf_min = 0;
  double filter_size_pcd = 0;
  double _first_lidar_time = 0.0;
  double match_time = 0, solve_time = 0, solve_const_H_time = 0;

  bool lidar_map_inited = false, save_all_pcd_en = false,
       save_all_img_en = false, pub_effect_point_en = false,
       pose_output_en = false, ros_driver_fix_en = false, hilti_en = false;
  int pcd_save_interval = -1, pcd_index = 0;
  int pub_scan_num = 1;

  StatesGroup imu_propagate, latest_ekf_state;

  bool new_imu = false, state_update_flg = false, imu_prop_enable = true,
       ekf_finish_once = false;
  deque<sensor_msgs::msg::Imu> prop_imu_buffer;  // ROS 2 消息类型
  sensor_msgs::msg::Imu newest_imu;
  double latest_ekf_time;
  nav_msgs::msg::Odometry imu_prop_odom;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubImuPropOdom;
  double imu_time_offset = 0.0;
  double lidar_time_offset = 0.0;

  bool gravity_align_en = false, gravity_align_finished = false;

  bool sync_jump_flag = false;
  bool flg_exit = false;

  bool lidar_pushed = false, imu_en, gravity_est_en, flg_reset = false,
       ba_bg_est_en = true;
  bool dense_map_en = false;
  int img_en = 1, imu_int_frame = 3;
  bool normal_en = true;
  bool exposure_estimate_en = false;
  double exposure_time_init = 0.0;
  bool inverse_composition_en = false;
  bool raycast_en = false;
  int lidar_en = 1;
  bool is_first_frame = false;
  int grid_size, patch_size, grid_n_width, grid_n_height, patch_pyrimid_level;
  double outlier_threshold;
  double plot_time;
  int frame_cnt;
  double img_time_offset = 0.0;
  deque<PointCloudXYZI::Ptr> lid_raw_data_buffer;
  deque<double> lid_header_time_buffer;
  deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer;
  deque<cv::Mat> img_buffer;
  deque<double> img_time_buffer;
  vector<pointWithVar> _pv_list;
  vector<double> extrinT;
  vector<double> extrinR;
  vector<double> cameraextrinT;
  vector<double> cameraextrinR;
  double IMG_POINT_COV;

  PointCloudXYZI::Ptr visual_sub_map;
  PointCloudXYZI::Ptr feats_undistort;
  PointCloudXYZI::Ptr feats_down_body;
  PointCloudXYZI::Ptr feats_down_world;
  PointCloudXYZI::Ptr pcl_w_wait_pub;
  PointCloudXYZI::Ptr pcl_wait_pub;
  PointCloudXYZRGB::Ptr pcl_wait_save;
  PointCloudXYZI::Ptr pcl_wait_save_intensity;

  V3D last_pcd_save_pos_;
  double pcd_save_distance_thresh_ = 0.2;
  bool is_first_pcd_saved_ = false;

  pcl::VoxelGrid<PointType> downSizeFilterSurf;

  V3D euler_cur;

  LidarMeasureGroup LidarMeasures;
  StatesGroup _state;
  StatesGroup state_propagat;

  nav_msgs::msg::Path path;
  nav_msgs::msg::Odometry odomAftMapped;
  geometry_msgs::msg::Quaternion geoQuat;
  geometry_msgs::msg::PoseStamped msg_body_pose;

  std::shared_ptr<Preprocess> p_pre;
  std::shared_ptr<ImuProcess> p_imu;
  VoxelMapManagerPtr voxelmap_manager;
  VIOManagerPtr vio_manager;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr plane_pub;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_pc;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr
      sub_pcl_livox;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pubLaserCloudFullRes;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pubLaserCloudEffect;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath;

  image_transport::Publisher pubImage;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      mavros_pose_publisher;

  rclcpp::TimerBase::SharedPtr imu_prop_timer;
  rclcpp::TimerBase::SharedPtr main_timer;

  // TF Broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  int frame_num = 0;
  double aver_time_consu = 0;
  double aver_time_icp = 0;
  double aver_time_map_inre = 0;
  bool colmap_output_en = false;

 private:
  PointCloudXYZI::Ptr filterPlanarPoints(
      const PointCloudXYZI::Ptr& input_cloud);

  std::string map_data_path_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_map_service_;

  bool auto_mode_enabled_ = false;
  double message_timeout_ = 5.0;
  bool auto_save_triggered_ = false;
  rclcpp::Time last_message_ros_time_;
  std::chrono::time_point<std::chrono::steady_clock> start_time_;

  static LIVMapper* ptr_;
};
#endif