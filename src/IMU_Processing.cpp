/*
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "IMU_Processing.h"

#include <cassert>

ImuProcess::ImuProcess()
    : Eye3d(M3D::Identity()),
      Zero3d(0, 0, 0),
      imu_need_init(true),
      b_first_frame(true) {  // Reordered
  init_iter_num = 1;
  cov_acc = V3D(0.1, 0.1, 0.1);
  cov_gyr = V3D(0.1, 0.1, 0.1);
  cov_bias_gyr = V3D(0.1, 0.1, 0.1);
  cov_bias_acc = V3D(0.1, 0.1, 0.1);
  cov_inv_expo = 0.2;
  mean_acc = V3D(0, 0, -1.0);
  mean_gyr = V3D(0, 0, 0);
  angvel_last = Zero3d;
  acc_s_last = Zero3d;
  Lid_offset_to_IMU = Zero3d;
  Lid_rot_to_IMU = Eye3d;
  last_imu.reset(new sensor_msgs::msg::Imu());
  cur_pcl_un_.reset(new PointCloudXYZI());
}

ImuProcess::~ImuProcess() {}

void ImuProcess::Reset() {
  LOG_WARN_F("Reset ImuProcess");
  mean_acc = V3D(0, 0, -1.0);
  mean_gyr = V3D(0, 0, 0);
  angvel_last = Zero3d;
  imu_need_init = true;
  init_iter_num = 1;
  IMUpose.clear();
  last_imu.reset(new sensor_msgs::msg::Imu());
  cur_pcl_un_.reset(new PointCloudXYZI());
}

void ImuProcess::disable_imu() {
  cout << "IMU Disabled !!!!!" << endl;
  imu_en = false;
  imu_need_init = false;
}

void ImuProcess::disable_gravity_est() {
  cout << "Online Gravity Estimation Disabled !!!!!" << endl;
  gravity_est_en = false;
}

void ImuProcess::disable_bias_est() {
  cout << "Bias Estimation Disabled !!!!!" << endl;
  ba_bg_est_en = false;
}

void ImuProcess::disable_exposure_est() {
  cout << "Online Time Offset Estimation Disabled !!!!!" << endl;
  exposure_estimate_en = false;
}

void ImuProcess::set_extrinsic(const MD(4, 4) & T) {
  Lid_offset_to_IMU = T.block<3, 1>(0, 3);
  Lid_rot_to_IMU = T.block<3, 3>(0, 0);
}

void ImuProcess::set_extrinsic(const V3D& transl) {
  Lid_offset_to_IMU = transl;
  Lid_rot_to_IMU.setIdentity();
}

void ImuProcess::set_extrinsic(const V3D& transl, const M3D& rot) {
  Lid_offset_to_IMU = transl;
  Lid_rot_to_IMU = rot;
}

void ImuProcess::set_gyr_cov_scale(const V3D& scaler) { cov_gyr = scaler; }

void ImuProcess::set_acc_cov_scale(const V3D& scaler) { cov_acc = scaler; }

void ImuProcess::set_gyr_bias_cov(const V3D& b_g) { cov_bias_gyr = b_g; }

void ImuProcess::set_inv_expo_cov(const double& inv_expo) {
  cov_inv_expo = inv_expo;
}

void ImuProcess::set_acc_bias_cov(const V3D& b_a) { cov_bias_acc = b_a; }

void ImuProcess::set_imu_init_frame_num(const int& num) { MAX_INI_COUNT = num; }

void ImuProcess::IMU_init(const MeasureGroup& meas, StatesGroup& state_inout,
                          int& N) {
  LOG_INFO_F("IMU Initializing: %.1f %%", double(N) / MAX_INI_COUNT * 100);
  V3D cur_acc, cur_gyr;

  if (b_first_frame) {
    Reset();
    N = 1;
    b_first_frame = false;
    const auto& imu_acc = meas.imu.front()->linear_acceleration;
    const auto& gyr_acc = meas.imu.front()->angular_velocity;
    mean_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    mean_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;
  }

  for (const auto& imu : meas.imu) {
    const auto& imu_acc = imu->linear_acceleration;
    const auto& gyr_acc = imu->angular_velocity;
    cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

    mean_acc += (cur_acc - mean_acc) / N;
    mean_gyr += (cur_gyr - mean_gyr) / N;

    N++;
  }
  IMU_mean_acc_norm = mean_acc.norm();
  state_inout.gravity = -mean_acc / mean_acc.norm() * G_m_s2;
  state_inout.rot_end = Eye3d;
  state_inout.bias_g = Zero3d;

  last_imu = meas.imu.back();
}

void ImuProcess::Forward_without_imu(LidarMeasureGroup& meas,
                                     StatesGroup& state_inout,
                                     PointCloudXYZI& pcl_out) {
  pcl_out = *(meas.lidar);
  const double& pcl_beg_time = meas.lidar_frame_beg_time;
  sort(pcl_out.points.begin(), pcl_out.points.end(), time_list);
  const double& pcl_end_time =
      pcl_beg_time + pcl_out.points.back().curvature / double(1000);
  meas.last_lio_update_time = pcl_end_time;
  const double& pcl_end_offset_time =
      pcl_out.points.back().curvature / double(1000);

  MD(DIM_STATE, DIM_STATE) F_x, cov_w;
  double dt = 0;

  if (b_first_frame) {
    dt = 0.1;
    b_first_frame = false;
  } else {
    dt = pcl_beg_time - time_last_scan;
  }

  time_last_scan = pcl_beg_time;

  M3D Exp_f = Exp(state_inout.bias_g, dt);

  F_x.setIdentity();
  cov_w.setZero();

  F_x.block<3, 3>(0, 0) = Exp(state_inout.bias_g, -dt);
  F_x.block<3, 3>(0, 10) = Eye3d * dt;
  F_x.block<3, 3>(3, 7) = Eye3d * dt;

  cov_w.block<3, 3>(10, 10).diagonal() = cov_gyr * dt * dt;
  cov_w.block<3, 3>(7, 7).diagonal() = cov_acc * dt * dt;

  state_inout.cov = F_x * state_inout.cov * F_x.transpose() + cov_w;
  state_inout.rot_end = state_inout.rot_end * Exp_f;
  state_inout.pos_end = state_inout.pos_end + state_inout.vel_end * dt;

  if (lidar_type != L515) {
    auto it_pcl = pcl_out.points.end() - 1;
    double dt_j = 0.0;
    for (; it_pcl != pcl_out.points.begin(); it_pcl--) {
      dt_j = pcl_end_offset_time - it_pcl->curvature / double(1000);
      M3D R_jk(Exp(state_inout.bias_g, -dt_j));
      V3D P_j(it_pcl->x, it_pcl->y, it_pcl->z);
      V3D p_jk;
      p_jk = -state_inout.rot_end.transpose() * state_inout.vel_end * dt_j;

      V3D P_compensate = R_jk * P_j + p_jk;

      it_pcl->x = P_compensate(0);
      it_pcl->y = P_compensate(1);
      it_pcl->z = P_compensate(2);
    }
  }
}

void ImuProcess::UndistortPcl(LidarMeasureGroup& lidar_meas,
                              StatesGroup& state_inout,
                              PointCloudXYZI& pcl_out) {
  pcl_out.clear();
  MeasureGroup& meas = lidar_meas.measures.back();
  auto v_imu = meas.imu;
  v_imu.push_front(last_imu);

  const double prop_beg_time = last_prop_end_time;
  const double prop_end_time =
      lidar_meas.lio_vio_flg == LIO ? meas.lio_time : meas.vio_time;

  if (lidar_meas.lio_vio_flg == LIO) {
    pcl_wait_proc.resize(lidar_meas.pcl_proc_cur->points.size());
    pcl_wait_proc = *(lidar_meas.pcl_proc_cur);
    lidar_meas.lidar_scan_index_now = 0;
    IMUpose.push_back(set_pose6d(0.0, acc_s_last, angvel_last,
                                 state_inout.vel_end, state_inout.pos_end,
                                 state_inout.rot_end));
  }

  V3D acc_imu(acc_s_last), angvel_avr(angvel_last), acc_avr,
      vel_imu(state_inout.vel_end), pos_imu(state_inout.pos_end);
  M3D R_imu(state_inout.rot_end);
  MD(DIM_STATE, DIM_STATE) F_x, cov_w;
  double dt = 0;
  double offs_t;
  double tau;

  if (!imu_time_init) {
    tau = 1.0;
    imu_time_init = true;
  } else {
    tau = state_inout.inv_expo_time;
  }

  if (lidar_meas.lio_vio_flg == LIO || lidar_meas.lio_vio_flg == VIO) {
    for (size_t i = 0; i < v_imu.size() - 1; ++i) {
      auto head = v_imu[i];
      auto tail = v_imu[i + 1];

      if (get_time_sec(tail->header.stamp) < prop_beg_time) continue;

      angvel_avr << 0.5 * (head->angular_velocity.x + tail->angular_velocity.x),
          0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
          0.5 * (head->angular_velocity.z + tail->angular_velocity.z);

      acc_avr << 0.5 * (head->linear_acceleration.x +
                        tail->linear_acceleration.x),
          0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
          0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z);

      fout_imu << setw(10)
               << get_time_sec(head->header.stamp) - first_lidar_time << " "
               << angvel_avr.transpose() << " " << acc_avr.transpose() << endl;

      angvel_avr -= state_inout.bias_g;
      acc_avr = acc_avr * G_m_s2 / mean_acc.norm() - state_inout.bias_a;

      if (get_time_sec(head->header.stamp) < prop_beg_time) {
        dt = get_time_sec(tail->header.stamp) - last_prop_end_time;
        offs_t = get_time_sec(tail->header.stamp) - prop_beg_time;
      } else if (i != v_imu.size() - 2) {
        dt =
            get_time_sec(tail->header.stamp) - get_time_sec(head->header.stamp);
        offs_t = get_time_sec(tail->header.stamp) - prop_beg_time;
      } else {
        dt = prop_end_time - get_time_sec(head->header.stamp);
        offs_t = prop_end_time - prop_beg_time;
      }

      M3D acc_avr_skew;
      M3D Exp_f = Exp(angvel_avr, dt);
      acc_avr_skew << SKEW_SYM_MATRX(acc_avr);

      F_x.setIdentity();
      cov_w.setZero();

      F_x.block<3, 3>(0, 0) = Exp(angvel_avr, -dt);
      if (ba_bg_est_en) F_x.block<3, 3>(0, 10) = -Eye3d * dt;
      F_x.block<3, 3>(3, 7) = Eye3d * dt;
      F_x.block<3, 3>(7, 0) = -R_imu * acc_avr_skew * dt;
      if (ba_bg_est_en) F_x.block<3, 3>(7, 13) = -R_imu * dt;
      if (gravity_est_en) F_x.block<3, 3>(7, 16) = Eye3d * dt;

      if (exposure_estimate_en) cov_w(6, 6) = cov_inv_expo * dt * dt;
      cov_w.block<3, 3>(0, 0).diagonal() = cov_gyr * dt * dt;
      cov_w.block<3, 3>(7, 7) =
          R_imu * cov_acc.asDiagonal() * R_imu.transpose() * dt * dt;
      cov_w.block<3, 3>(10, 10).diagonal() = cov_bias_gyr * dt * dt;
      cov_w.block<3, 3>(13, 13).diagonal() = cov_bias_acc * dt * dt;

      state_inout.cov = F_x * state_inout.cov * F_x.transpose() + cov_w;

      R_imu = R_imu * Exp_f;
      acc_imu = R_imu * acc_avr + state_inout.gravity;
      pos_imu = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt;
      vel_imu = vel_imu + acc_imu * dt;

      angvel_last = angvel_avr;
      acc_s_last = acc_imu;

      IMUpose.push_back(
          set_pose6d(offs_t, acc_imu, angvel_avr, vel_imu, pos_imu, R_imu));
    }
    lidar_meas.last_lio_update_time = prop_end_time;
  }

  state_inout.vel_end = vel_imu;
  state_inout.rot_end = R_imu;
  state_inout.pos_end = pos_imu;
  state_inout.inv_expo_time = tau;

  last_imu = v_imu.back();
  last_prop_end_time = prop_end_time;

  if (pcl_wait_proc.points.size() < 1) return;

  if (lidar_meas.lio_vio_flg == LIO) {
    auto it_pcl = pcl_wait_proc.points.end() - 1;
    M3D extR_Ri(Lid_rot_to_IMU.transpose() * state_inout.rot_end.transpose());
    V3D exrR_extT(Lid_rot_to_IMU.transpose() * Lid_offset_to_IMU);
    for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--) {
      auto head = it_kp - 1;
      R_imu << MAT_FROM_ARRAY(head->rot);
      acc_imu << VEC_FROM_ARRAY(head->acc);
      vel_imu << VEC_FROM_ARRAY(head->vel);
      pos_imu << VEC_FROM_ARRAY(head->pos);
      angvel_avr << VEC_FROM_ARRAY(head->gyr);

      for (; it_pcl->curvature / double(1000) > head->offset_time; it_pcl--) {
        dt = it_pcl->curvature / double(1000) - head->offset_time;

        M3D R_i(R_imu * Exp(angvel_avr, dt));
        V3D T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt -
                 state_inout.pos_end);

        V3D P_i(it_pcl->x, it_pcl->y, it_pcl->z);
        V3D P_compensate =
            (extR_Ri *
                 (R_i * (Lid_rot_to_IMU * P_i + Lid_offset_to_IMU) + T_ei) -
             exrR_extT);

        it_pcl->x = P_compensate(0);
        it_pcl->y = P_compensate(1);
        it_pcl->z = P_compensate(2);

        if (it_pcl == pcl_wait_proc.points.begin()) break;
      }
    }
    pcl_out = pcl_wait_proc;
    pcl_wait_proc.clear();
    IMUpose.clear();
  }
}

void ImuProcess::Process2(LidarMeasureGroup& lidar_meas, StatesGroup& stat,
                          PointCloudXYZI::Ptr cur_pcl_un_) {
  assert(lidar_meas.lidar != nullptr);
  if (!imu_en) {
    Forward_without_imu(lidar_meas, stat, *cur_pcl_un_);
    return;
  }

  MeasureGroup meas = lidar_meas.measures.back();

  if (imu_need_init) {
    if (meas.imu.empty()) {
      return;
    };
    /// The very first lidar frame
    IMU_init(meas, stat, init_iter_num);

    imu_need_init = true;

    last_imu = meas.imu.back();

    if (init_iter_num > MAX_INI_COUNT) {
      imu_need_init = false;
      LOG_INFO_F(
          "IMU Initials: Gravity: %.4f %.4f %.4f %.4f; acc covarience: "
          "%.8f %.8f %.8f; gry covarience: %.8f %.8f %.8f \n",
          stat.gravity[0], stat.gravity[1], stat.gravity[2], mean_acc.norm(),
          cov_acc[0], cov_acc[1], cov_acc[2], cov_gyr[0], cov_gyr[1],
          cov_gyr[2]);
      LOG_INFO_F(
          "IMU Initials: ba covarience: %.8f %.8f %.8f; bg covarience: "
          "%.8f %.8f %.8f",
          cov_bias_acc[0], cov_bias_acc[1], cov_bias_acc[2], cov_bias_gyr[0],
          cov_bias_gyr[1], cov_bias_gyr[2]);
      fout_imu.open(DEBUG_FILE_DIR("imu.txt"), ios::out);
    }

    return;
  }

  UndistortPcl(lidar_meas, stat, *cur_pcl_un_);
}

void ImuProcess::Reset(const StatesGroup& state, double start_timestamp) {
  angvel_last = state.bias_g;
  acc_s_last = state.rot_end.transpose() * (V3D(0, 0, -G_m_s2) - state.gravity);

  last_prop_end_time = start_timestamp;

  sensor_msgs::msg::Imu::SharedPtr temp_imu =
      std::make_shared<sensor_msgs::msg::Imu>();
  temp_imu->header.stamp = get_ros_time(start_timestamp);
  last_imu = temp_imu;

  IMUpose.clear();

  imu_need_init = false;
}