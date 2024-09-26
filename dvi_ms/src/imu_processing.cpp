#include "imu_processing.h"

ImuProcessing::ImuProcessing() 
: b_first_frame_(true), imu_need_init_(true), start_timestamp_(-1) {
  imu_en = true;
  init_iter_num = 1;
  cov_acc = V3D(0.1, 0.1, 0.1);
  cov_gyr = V3D(0.1, 0.1, 0.1);
  // old
  cov_acc_scale = V3D(1, 1, 1);
  cov_gyr_scale = V3D(1, 1, 1);

  cov_bias_gyr = V3D(0.1, 0.1, 0.1);
  cov_bias_acc = V3D(0.1, 0.1, 0.1);
  mean_acc = V3D(0, 0, -1.0);
  mean_gyr = V3D(0, 0, 0);
  angvel_last = Zero3d;
  Lid_offset_to_IMU = Zero3d;
  Lid_rot_to_IMU = Eye3d;
  imu_border_after.reset(new sensor_msgs::Imu());
  imu_border_before.reset(new sensor_msgs::Imu());
}

ImuProcessing::~ImuProcessing() {}


void ImuProcessing::reset() {
  ROS_WARN("Reset ImuProcess");
  mean_acc = V3D(0, 0, -1.0);
  mean_gyr = V3D(0, 0, 0);
  angvel_last = Zero3d;
  imu_need_init_ = true;
  start_timestamp_ = -1;
  init_iter_num = 1;
  v_imu_.clear();
  IMUpose.clear();
  imu_border_after.reset(new sensor_msgs::Imu());
  imu_border_before.reset(new sensor_msgs::Imu());
  cur_pcl_un_.reset(new PointCloudType());
}

void ImuProcessing::set_extrinsic(const MD(4, 4) & T) {
  Lid_offset_to_IMU = T.block<3, 1>(0, 3);
  Lid_rot_to_IMU = T.block<3, 3>(0, 0);
}

void ImuProcessing::set_extrinsic(const V3D &transl) {
  Lid_offset_to_IMU = transl;
  Lid_rot_to_IMU.setIdentity();
}

void ImuProcessing::set_extrinsic(const V3D &transl, const M3D &rot) {
  Lid_offset_to_IMU = transl;
  Lid_rot_to_IMU = rot;
}

void ImuProcessing::set_gyr_cov_scale(const V3D &scaler) {
  cov_gyr_scale = scaler;
}

void ImuProcessing::set_acc_cov_scale(const V3D &scaler) {
  cov_acc_scale = scaler;
}

void ImuProcessing::set_gyr_bias_cov(const V3D &b_g) { cov_bias_gyr = b_g; }

void ImuProcessing::set_acc_bias_cov(const V3D &b_a) { cov_bias_acc = b_a; }

void ImuProcessing::IMU_init(deque<sensor_msgs::Imu::ConstPtr> &msgs_imu, StatesGroup &state_inout, int &N) {
  /** 1. initializing the gravity, gyro bias, acc and gyro covariance
   ** 2. normalize the acceleration measurenments to unit gravity **/
  ROS_INFO("[propagation] IMU Initializing: %.1f %%", double(N) / MAX_INI_COUNT * 100);
  V3D cur_acc, cur_gyr;

  if (b_first_frame_) {
    reset();
    N = 1;
    b_first_frame_ = false;
    const auto &imu_acc = msgs_imu.front()->linear_acceleration;
    const auto &gyr_acc = msgs_imu.front()->angular_velocity;
    mean_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    mean_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;
    // cout<<"init acc norm: "<<mean_acc.norm()<<endl;
  }

  for (const auto &imu : msgs_imu) {
    const auto &imu_acc = imu->linear_acceleration;
    const auto &gyr_acc = imu->angular_velocity;
    cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

    mean_acc += (cur_acc - mean_acc) / N;
    mean_gyr += (cur_gyr - mean_gyr) / N;

    cov_acc = cov_acc * (N - 1.0) / N +
              (cur_acc - mean_acc).cwiseProduct(cur_acc - mean_acc) *
                  (N - 1.0) / (N * N);
    cov_gyr = cov_gyr * (N - 1.0) / N +
              (cur_gyr - mean_gyr).cwiseProduct(cur_gyr - mean_gyr) *
                  (N - 1.0) / (N * N);

    // cout<<"acc norm: "<<cur_acc.norm()<<" "<<mean_acc.norm()<<endl;

    N++;
  }

  state_inout.gravity = -mean_acc / mean_acc.norm() * G_m_s2;
  state_inout.rot_end = Eye3d; // Exp(mean_acc.cross(V3D(0, 0, -1 / scale_gravity)));
  state_inout.bias_g = mean_gyr;

  imu_border_after = msgs_imu.back();
}


void ImuProcessing::state_propagation(deque<sensor_msgs::Imu::ConstPtr> &msgs_imu,
                              StatesGroup &state_inout,
                              VILAINPublisher &publisher,
                              double lidar_time_previous, 
                              double lidar_time_current) {
                                
  // add the imu of the last frame-tail to the of current frame-head 
  auto v_imu = msgs_imu;
  v_imu.push_front(imu_border_after);
  v_imu.push_front(imu_border_before);
  const double &imu_beg_time = v_imu.front()->header.stamp.toSec();
  const double &imu_end_time = v_imu.back()->header.stamp.toSec();
  const double &pcl_beg_time = lidar_time_previous;
  // ROS_INFO("times: %.6f, %.6f, %.6f", imu_beg_time, imu_end_time, pcl_beg_time);
  if(imu_beg_time > lidar_time_previous) {
    ROS_ERROR("IMU-measurement sync: imu begins after previous lidar");
  }
  if(imu_end_time < lidar_time_current) {
    ROS_ERROR("IMU-measurement sync: imu ends before current lidar");
  }

  // init IMU pose for these state propagations
  positions_state_propagation.clear();
  rotations_state_propagation.clear();
  IMUpose.push_back(set_pose6d(0.0, acc_s_last, angvel_last,
                               state_inout.vel_end, state_inout.pos_end,
                               state_inout.rot_end));
  positions_state_propagation.push_back(state_inout.pos_end);
  rotations_state_propagation.push_back(state_inout.rot_end);

  // forward propagation at each imu data point
  V3D acc_imu, angvel_avr, acc_avr, vel_imu(state_inout.vel_end), pos_imu(state_inout.pos_end);
  M3D R_imu(state_inout.rot_end);
  MD(DIM_STATE, DIM_STATE) F_x, cov_w;

  double dt = 0;
  int if_first_imu = 1;
  for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++) {
    auto &&head = *(it_imu);
    auto &&tail = *(it_imu + 1);
    // ROS_INFO("propagating state at IMU timestamp %.6f", head->header.stamp.toSec());

    // CH perhaps check if we're not getting old IMU data here
    // if (tail->header.stamp.toSec() < pcl_beg_time)
    // continue;

    angvel_avr << 0.5 * (head->angular_velocity.x + tail->angular_velocity.x),
        0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
        0.5 * (head->angular_velocity.z + tail->angular_velocity.z);
    acc_avr << 0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x),
        0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
        0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z);

    angvel_avr -= state_inout.bias_g;
    acc_avr = acc_avr * G_m_s2 / mean_acc.norm() - state_inout.bias_a;

    // CH to do 
    // something to be changed with different model
    dt = tail->header.stamp.toSec() - head->header.stamp.toSec();
    /*
    if (if_first_imu)
    {
      if_first_imu = 0;
      dt = tail->header.stamp.toSec() - lidar_time_previous;
    }
    else {
      dt = tail->header.stamp.toSec() - head->header.stamp.toSec();
    }*/
    if (dt > 0.05)  {
      ROS_ERROR("ERROR: to big dt %f", dt);
      dt = 0.05;
    }
    // ROS_INFO("IMU state propagation dt: %f", dt);
    /* covariance propagation */
    M3D acc_avr_skew;
    M3D Exp_f = Exp(angvel_avr, dt);
    acc_avr_skew << SKEW_SYM_MATRX(acc_avr);

    F_x.setIdentity();
    cov_w.setZero();

    F_x.block<3, 3>(0, 0) = Exp(angvel_avr, -dt);
    F_x.block<3, 3>(0, 9) = -Eye3d * dt; // CH this is not correct
    // F_x.block<3,3>(3,0)  = R_imu * off_vel_skew * dt;
    F_x.block<3, 3>(3, 6) = Eye3d * dt;
    F_x.block<3, 3>(6, 0) = -R_imu * acc_avr_skew * dt;
    F_x.block<3, 3>(6, 12) = -R_imu * dt;
    F_x.block<3, 3>(6, 15) = Eye3d * dt;

    cov_w.block<3, 3>(0, 0).diagonal() = cov_gyr * dt * dt;
    cov_w.block<3, 3>(6, 6) =  R_imu * cov_acc.asDiagonal() * R_imu.transpose() * dt * dt;
    cov_w.block<3, 3>(9, 9).diagonal() = cov_bias_gyr * dt * dt; // bias gyro covariance
    cov_w.block<3, 3>(12, 12).diagonal() = cov_bias_acc * dt * dt; // bias acc covariance

    state_inout.cov = F_x * state_inout.cov * F_x.transpose() + cov_w;

    // propagation of state variables (in global frame)
    R_imu = R_imu * Exp_f;
    acc_imu = R_imu * acc_avr + state_inout.gravity;
    pos_imu = pos_imu + vel_imu * dt; //CH + 0.5 * acc_imu * dt * dt;
    vel_imu = vel_imu + acc_imu * dt;


    /* save the poses at each IMU measurements */
    angvel_last = angvel_avr;
    acc_s_last = acc_imu;
    double &&offs_t = tail->header.stamp.toSec() - pcl_beg_time;
    positions_state_propagation.push_back(pos_imu);
    rotations_state_propagation.push_back(R_imu);
  }
  // ROS_INFO("imu path length %d", positions_state_propagation.size());
  publisher.publish_state_propagation(positions_state_propagation, rotations_state_propagation);

  /*** calculated the pos and attitude prediction at the frame-end ***/
  double note = lidar_time_current > imu_end_time ? 1.0 : -1.0;
  dt = note * (lidar_time_current - imu_end_time);
  // ROS_INFO("IMU last dt: %f", dt);
  state_inout.vel_end = vel_imu + note * acc_imu * dt;
  state_inout.rot_end = R_imu * Exp(V3D(note * angvel_avr), dt);
  state_inout.pos_end = pos_imu + note * vel_imu * dt + note * 0.5 * acc_imu * dt * dt;
  
}

// constant velocity model
void ImuProcessing::state_propagation_constant_velocity(double pcl_beg_time, StatesGroup &state_inout) {

  MD(DIM_STATE, DIM_STATE) F_x, cov_w;
  double dt = 0;

  if (b_first_frame_) {
    dt = 0.1;
    b_first_frame_ = false;
    time_last_scan_ = pcl_beg_time;
  } else {
    dt = pcl_beg_time - time_last_scan_;
    time_last_scan_ = pcl_beg_time;
  }
  std::cout << "dt " <<  dt <<  std::endl;
  if(dt > 0.1) {
    ROS_ERROR("[propagation] too big dt, skipping");
    return;
  }

  ROS_INFO("[propagation] constant velocity model - state before");
  /* std::cout << "rot" << std::endl;
  std::cout << state_inout.rot_end << std::endl;
  std::cout << "pos" << std::endl;
  std::cout << state_inout.pos_end << std::endl; */

  /* covariance propagation */
  // M3D acc_avr_skew;
  M3D Exp_f = Exp(state_inout.bias_g, dt);

  F_x.setIdentity();
  cov_w.setZero();

  F_x.block<3, 3>(0, 0) = Exp(state_inout.bias_g, -dt);
  F_x.block<3, 3>(0, 9) = Eye3d * dt;
  F_x.block<3, 3>(3, 6) = Eye3d * dt;
  cov_w.block<3, 3>(9, 9).diagonal() = cov_gyr * dt * dt; // for omega in constant model
  cov_w.block<3, 3>(6, 6).diagonal() = cov_acc * dt * dt; // for velocity in constant model

  state_inout.cov = F_x * state_inout.cov * F_x.transpose() + cov_w;
  state_inout.rot_end = state_inout.rot_end * Exp_f;
  state_inout.pos_end = state_inout.pos_end + state_inout.vel_end * dt;

  ROS_INFO("[propagation] constant velocity model - state after");
  /* std::cout << "rot" << std::endl;
  std::cout << state_inout.rot_end << std::endl;
  std::cout << "pos" << std::endl;
  std::cout << state_inout.pos_end << std::endl; */
}
/*
void ImuProcessing::state_propagation_constant_velocity(const MeasureGroup &meas, StatesGroup &state_inout,
                             PointCloudType::Ptr &pcl_out) {

  const double &pcl_beg_time = meas.lidar_beg_time;
  pcl_out = meas.lidar;
  const double &pcl_end_time = pcl_beg_time + pcl_out->points.back().curvature / double(1000);

  MD(DIM_STATE, DIM_STATE) F_x, cov_w;
  double dt = 0;

  if (b_first_frame_) {
    dt = 0.1;
    b_first_frame_ = false;
    time_last_scan_ = pcl_beg_time;
  } else {
    dt = pcl_beg_time - time_last_scan_;
    time_last_scan_ = pcl_beg_time;
  }

  // covariance propagation 
  // M3D acc_avr_skew;
  M3D Exp_f = Exp(state_inout.bias_g, dt);

  F_x.setIdentity();
  cov_w.setZero();

  F_x.block<3, 3>(0, 0) = Exp(state_inout.bias_g, -dt);
  F_x.block<3, 3>(0, 9) = Eye3d * dt;
  F_x.block<3, 3>(3, 6) = Eye3d * dt;
  cov_w.block<3, 3>(9, 9).diagonal() = cov_gyr * dt * dt; // for omega in constant model
  cov_w.block<3, 3>(6, 6).diagonal() = cov_acc * dt * dt; // for velocity in constant model

  state_inout.cov = F_x * state_inout.cov * F_x.transpose() + cov_w;
  state_inout.rot_end = state_inout.rot_end * Exp_f;
  state_inout.pos_end = state_inout.pos_end + state_inout.vel_end * dt;
}
*/
/*
void ImuProcessing::process(const MeasureGroup &meas, StatesGroup &stat,
                         PointCloudType::Ptr &cur_pcl_un_,
                        double lidar_time_previous, double lidar_time_current,
                              Publisher &publisher) {

  if (meas.imu.empty() && imu_en) {
    return;
  }
  ROS_ASSERT(meas.lidar != nullptr);

  if (imu_need_init_ && imu_en) {
    /// The very first lidar frame
    IMU_init(meas, stat, init_iter_num);

    imu_need_init_ = true;

    last_imu_ = meas.imu.back();

    if (init_iter_num > MAX_INI_COUNT) {
      cov_acc *= pow(G_m_s2 / mean_acc.norm(), 2);
      imu_need_init_ = false;
      ROS_INFO("IMU Initials: Gravity: %.4f %.4f %.4f %.4f; state.bias_g: %.4f "
               "%.4f %.4f; acc covarience: %.8f %.8f %.8f; gry covarience: "
               "%.8f %.8f %.8f",
               stat.gravity[0], stat.gravity[1], stat.gravity[2],
               mean_acc.norm(), cov_acc_scale[0], cov_acc_scale[1],
               cov_acc_scale[2], cov_acc[0], cov_acc[1], cov_acc[2], cov_gyr[0],
               cov_gyr[1], cov_gyr[2]);
      cov_acc = Eye3d * cov_acc_scale;
      cov_gyr = Eye3d * cov_gyr_scale;
      // cout<<"mean acc: "<<mean_acc<<" acc measures in word
      // frame:"<<state.rot_end.transpose()*mean_acc<<endl;
      ROS_INFO("IMU Initials: Gravity: %.4f %.4f %.4f %.4f; state.bias_g: %.4f "
               "%.4f %.4f; acc covarience: %.8f %.8f %.8f; gry covarience: "
               "%.8f %.8f %.8f",
               stat.gravity[0], stat.gravity[1], stat.gravity[2],
               mean_acc.norm(), cov_bias_gyr[0], cov_bias_gyr[1],
               cov_bias_gyr[2], cov_acc[0], cov_acc[1], cov_acc[2], cov_gyr[0],
               cov_gyr[1], cov_gyr[2]);
    }

    return;
  }

  /// Undistort points： the first point is assummed as the base frame
  /// Compensate lidar points with IMU rotation (with only rotation now)
  if (imu_en) {
    cout << "Use IMU" << endl;
    state_propagation(meas, stat, *cur_pcl_un_, publisher, lidar_time_previous, lidar_time_current);
    last_imu_ = meas.imu.back();
  } else {
    cout << "No IMU, use constant velocity model" << endl;
    cov_acc = Eye3d * cov_acc_scale;
    cov_gyr = Eye3d * cov_gyr_scale;
    state_propagation_constant_velocity(meas, stat, cur_pcl_un_);
  }
}
*/

void ImuProcessing::state_propagation_imu(
  deque<sensor_msgs::Imu::ConstPtr> &msgs_imu, 
  StatesGroup &stat, 
  double time_measurement_previous, 
  double time_measurement_current, 
  VILAINPublisher &publisher) {

  if (imu_need_init_ && imu_en) {
    /// The very first lidar frame
    IMU_init(msgs_imu, stat, init_iter_num);

    imu_need_init_ = true;

    imu_border_after = msgs_imu.back();

    if (init_iter_num > MAX_INI_COUNT) {
      cov_acc *= pow(G_m_s2 / mean_acc.norm(), 2);
      imu_need_init_ = false;
      ROS_INFO("[propagation] IMU Initials: Gravity: %.4f %.4f %.4f %.4f; state.bias_g: %.4f "
               "%.4f %.4f; acc covariance: %.8f %.8f %.8f; gry covariance: "
               "%.8f %.8f %.8f",
               stat.gravity[0], stat.gravity[1], stat.gravity[2],
               mean_acc.norm(), cov_acc_scale[0], cov_acc_scale[1],
               cov_acc_scale[2], cov_acc[0], cov_acc[1], cov_acc[2], cov_gyr[0],
               cov_gyr[1], cov_gyr[2]);
      cov_acc = Eye3d * cov_acc_scale;
      cov_gyr = Eye3d * cov_gyr_scale;
      // cout<<"mean acc: "<<mean_acc<<" acc measures in word
      // frame:"<<state.rot_end.transpose()*mean_acc<<endl;
      ROS_INFO("[propagation] IMU Initials: Gravity: %.4f %.4f %.4f %.4f; state.bias_g: %.4f "
               "%.4f %.4f; acc covariance: %.8f %.8f %.8f; gry covariance: "
               "%.8f %.8f %.8f",
               stat.gravity[0], stat.gravity[1], stat.gravity[2],
               mean_acc.norm(), cov_bias_gyr[0], cov_bias_gyr[1],
               cov_bias_gyr[2], cov_acc[0], cov_acc[1], cov_acc[2], cov_gyr[0],
               cov_gyr[1], cov_gyr[2]);
    }

    return;
  }


    state_propagation(msgs_imu, stat, publisher, time_measurement_previous, time_measurement_current);
    imu_border_before = msgs_imu.at(msgs_imu.size() - 2);
    imu_border_after = msgs_imu.back();
}
