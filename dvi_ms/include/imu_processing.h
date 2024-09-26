#pragma once 

#include "state.h"
#include "vilain_publisher.h"
#include <Eigen/Eigen>
#include <cmath>
#include <common_lib.h>
#include <condition_variable>
#include <csignal>
#include <deque>
#include <eigen_conversions/eigen_msg.h>
#include <fstream>
#include <geometry_msgs/Vector3.h>
#include <math.h>
#include <mutex>
#include <nav_msgs/Odometry.h>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <so3_math.h>
#include <tf/transform_broadcaster.h>
#include <thread>
#include <dvi_ms/States.h>

/// *************Preconfiguration

#define MAX_INI_COUNT (200)

/// *************IMU Process and undistortion
class ImuProcessing {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuProcessing();
  ~ImuProcessing();


  void reset();
  void reset(double start_timestamp, const sensor_msgs::ImuConstPtr &lastimu);
  void set_extrinsic(const V3D &transl, const M3D &rot);
  void set_extrinsic(const V3D &transl);
  void set_extrinsic(const MD(4, 4) & T);
  void set_gyr_cov_scale(const V3D &scaler);
  void set_acc_cov_scale(const V3D &scaler);
  void set_gyr_bias_cov(const V3D &b_g);
  void set_acc_bias_cov(const V3D &b_a);
  void process(
    const MeasureGroup &meas, 
    StatesGroup &state, 
    PointCloudType::Ptr &pcl_un_,
    double lidar_time_previous,
    double lidar_time_current,
    VILAINPublisher& publisher);
  void state_propagation_imu(
    deque<sensor_msgs::Imu::ConstPtr> &msgs_imu, 
    StatesGroup &stat, 
    double time_measurement_previous, 
    double time_measurement_current, 
    VILAINPublisher &publisher);
  void state_propagation_constant_velocity(double pcl_beg_time, StatesGroup &state_inout);

  ofstream fout_imu;
  V3D cov_acc;
  V3D cov_gyr;
  V3D cov_acc_scale;
  V3D cov_gyr_scale;
  V3D cov_bias_gyr;
  V3D cov_bias_acc;
  M3D Lid_rot_to_IMU;
  V3D Lid_offset_to_IMU;
  double first_lidar_time;
  bool imu_en;

private:
  void IMU_init(deque<sensor_msgs::Imu::ConstPtr> &msgs_imu, StatesGroup &state, int &N);
  void state_propagation(
    deque<sensor_msgs::Imu::ConstPtr> &msgs_imu, 
    StatesGroup &state_inout, 
    VILAINPublisher& publisher,
    double lidar_time_previous,
    double lidar_time_current);
    

  PointCloudType::Ptr cur_pcl_un_;
  sensor_msgs::ImuConstPtr imu_border_after;
  sensor_msgs::ImuConstPtr imu_border_before;
  deque<sensor_msgs::ImuConstPtr> v_imu_;
  vector<Pose6D> IMUpose;
  vector<V3D> positions_state_propagation;
  vector<M3D> rotations_state_propagation;
  vector<M3D> v_rot_pcl_;

  V3D mean_acc;
  V3D mean_gyr;
  V3D angvel_last;
  V3D acc_s_last;
  double start_timestamp_;
  double time_last_scan_;
  int init_iter_num = 1;
  bool b_first_frame_ = true;
  bool imu_need_init_ = true;
};