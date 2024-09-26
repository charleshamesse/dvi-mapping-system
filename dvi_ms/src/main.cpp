#include "imu_processing.h"
#include "lidar_processing.h"
#include "state.h"
#include "voxel_mapping.h"
#include "parameters.h"
#include "vilain_publisher.h"
#include "vio_processing.h"
#include "transformer.h"
#include "common_lib.h"
#include <limits>
#include <Eigen/Core>
#include <csignal>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <image_transport/image_transport.h>
#include <math.h>
#include <mutex>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <opencv2/opencv.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>
#include <so3_math.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf2_msgs/TFMessage.h>
#include <thread>
#include <unistd.h>
#include <dvi_ms/States.h>
#include <degeneracy_handler.h>
#include <opencv2/core/eigen.hpp>


StatesGroup state,
            state_propagated;
VD(DIM_STATE) solution,
              solution_mask;
MD(DIM_STATE, DIM_STATE) G, H_T_H, I_STATE;
V3D rot_add, t_add, euler_cur, position_last;
geometry_msgs::Quaternion geoQuat;

std::unordered_map<VOXEL_LOC, OctoTree *> vx_map;

deque<sensor_msgs::PointCloud2::ConstPtr> buffer_lidar;
deque<sensor_msgs::Imu::ConstPtr> buffer_imu;
deque<pair<sensor_msgs::PointCloud2::ConstPtr, geometry_msgs::PoseWithCovarianceStamped::ConstPtr>> buffer_sync;

deque<double> time_buffer_lidar;
deque<double> time_buffer_imu;
deque<double> time_buffer_sync;

double  time_last_correction = -1.0f,
        res_mean_last = 0.05,
        total_distance = 0;

shared_ptr<ImuProcessing> p_imu(nullptr);
shared_ptr<LidarProcessing> p_lidar(nullptr);
shared_ptr<VioProcessing> p_vio(nullptr);
shared_ptr<Transformer> p_transformer(nullptr);
shared_ptr<VILAINPublisher> p_publisher(nullptr);
shared_ptr<DegeneracyHandler> p_degeneracy_handler(nullptr);

bool  flag_exit = false,
      flag_reset = false,
      vio_initialized = false,
      esikf_initialized = false,
      is_first_frame = true,
      map_initialized = false,
      EKF_stop_flg = false,
      flg_EKF_converged = false;

condition_variable sig_buffer;
mutex mtx_buffer;

int scanIdx = 0,
    effct_feat_num;

double  time_solve,
        first_lidar_time,
        scan_match_time,
        map_incremental_time,
        time_global_delta;

deque<sensor_msgs::Imu::ConstPtr> buffer_imu_current;

std::deque<std::tuple<double, Eigen::Matrix4d, Eigen::Matrix<double, 6, 6>>> poses_final;

PointCloudType::Ptr feats_undistorted(new PointCloudType());
pcl::PointCloud<pcl::PointXYZINormal>::Ptr feats_undistorted_copy(new pcl::PointCloud<pcl::PointXYZINormal>());
PointCloudXYZI::Ptr feats_downsampled_body(new PointCloudXYZI());
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr laserCloudNoeffect(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));
pcl::VoxelGrid<PointType> downSizeFilterSurf;

// degeneration
VD(6) eigen_values_HT_H;

// to be refactored:
ros::Publisher pub_voxel_map;

// methods
void SigHandle(int sig) {
  flag_exit = true;
  ROS_WARN("catch sig %d", sig);
  sig_buffer.notify_all();
}

void callback_cloud(const sensor_msgs::PointCloud2::ConstPtr &msg) {
  mtx_buffer.lock();

  // check for loopback
  if(time_buffer_lidar.size() > 1) {
    if (msg->header.stamp.toSec() < time_buffer_lidar.back()) {
      ROS_ERROR("[data] cloud loopback, clearing buffer");
      buffer_lidar.clear();
    }
  }
  ROS_INFO("[data] received cloud\t%.6f", msg->header.stamp.toSec());
  
  buffer_lidar.push_back(msg);
  time_buffer_lidar.push_back(msg->header.stamp.toSec());

  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

void callback_imu(const sensor_msgs::Imu::ConstPtr &msg_in) {
  mtx_buffer.lock();

  // check for loopback
  sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));
  double timestamp = msg->header.stamp.toSec();
  if(time_buffer_imu.size() > 1) {
    if (timestamp < time_buffer_imu.back()) {
      ROS_ERROR("[data] IMU loopback, clearing buffer");
      buffer_imu.clear();
    }
  }
  // ROS_INFO("Data received: IMU\t%.6f", msg->header.stamp.toSec());
  
  buffer_imu.push_back(msg);
  time_buffer_imu.push_back(timestamp);

  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

float time_last = 0;
void callback_cloud_and_pose(const sensor_msgs::PointCloud2ConstPtr& pc, const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose)
{
  mtx_buffer.lock();

  ROS_INFO("[data] got sync data %f, %f", pc->header.stamp.toSec(), pose->header.stamp.toSec());
  ROS_INFO("[data]                %f", pc->header.stamp.toSec() - pose->header.stamp.toSec());
  if(time_last > 1.f) {
    float time_since_last = pc->header.stamp.toSec() - time_last;
    ROS_INFO("[data]                %f", time_since_last);
  }
  time_last = pc->header.stamp.toSec();
  pair<sensor_msgs::PointCloud2::ConstPtr, geometry_msgs::PoseWithCovarianceStamped::ConstPtr> pair_sync;
  pair_sync.first = pc;
  pair_sync.second = pose;
  buffer_sync.push_back(pair_sync);
  time_buffer_sync.push_back(pose->header.stamp.toSec());

  mtx_buffer.unlock();
  sig_buffer.notify_all();
}
bool propagate(double time_measurement, StatesGroup &state) {
  // if using IMU
  if(enable_imu_propagation) {
    // check if there's IMU data
    if(buffer_imu.empty()) return false; // continue
    // ROS_INFO("[propagation]   propagating with IMU inside %f, %f", buffer_imu.back()->header.stamp.toSec(), time_measurement);

    // check if we still need to wait: in case last IMU is before measurement
    if(buffer_imu.back()->header.stamp.toSec() < time_measurement) return false;

    buffer_imu_current.clear();
    double time_imu = buffer_imu.front()->header.stamp.toSec();
    // go through IMU buffer from beginning until (first of both) measurement(s)
    while ((!buffer_imu.empty()) && (time_imu < time_measurement)) {
      time_imu = buffer_imu.front()->header.stamp.toSec();

      // if it's after measurement by some margin, stop adding
      if (time_imu > time_measurement + 0.02)
        break;
      
      buffer_imu_current.push_back(buffer_imu.front());
      buffer_imu.pop_front();
      time_buffer_imu.pop_front();
    }

    if(buffer_imu_current.empty()) {
      ROS_INFO("[propagation] time measurement %.6f, time imu %.6f", time_measurement, time_imu);
    
      ROS_INFO("[propagation] first imu is after measurement - this happens with sync lidar-vins-mono - removing first synced item");
      if(buffer_sync.size() > 0) {
        buffer_sync.pop_front();
        time_buffer_sync.pop_front();
      }
      if(buffer_lidar.size() > 0) {
        buffer_lidar.pop_front();
        time_buffer_lidar.pop_front();
      }
      return false; // continue
    }
    // put back last in buffer (so that t_imu_fist < t_lidar_prev < l_lidar_next < t_imu_last)
    buffer_imu.push_front(buffer_imu_current.back());
    time_buffer_imu.push_front(buffer_imu_current.back()->header.stamp.toSec());
    
    ROS_INFO("[propagation]   propagating with IMU from %.6f until %.6f", buffer_imu_current.front()->header.stamp.toSec(), buffer_imu_current.back()->header.stamp.toSec());
    // ROS_INFO("[propagation]   measurement is at                    %.6f", time_measurement);

    if(time_last_correction < 0) time_last_correction = buffer_imu_current.front()->header.stamp.toSec();
    p_imu->state_propagation_imu(buffer_imu_current, state, time_last_correction, time_measurement, *p_publisher);
    return true;
  }
  // otherwise use constant velocity model
  else {
    ROS_INFO("[propagation]   propagating with constant velocity model");
    p_imu->state_propagation_constant_velocity(time_last_correction, state);
    return true;
  }
}

const bool var_contrast(pointWithCov &x, pointWithCov &y) {
  return (x.cov.diagonal().norm() < y.cov.diagonal().norm());
};


bool correct_lidar(double time_measurement, StatesGroup &state, const PointCloudType::Ptr & feats_undistort) {
  ROS_INFO("[correction-stm] correcting state with scan-to-map at time: %.6f", time_measurement);
  // PointCloudType::Ptr feats_undistort = feats_undistorted.makeShared();

  if (flag_reset) {
    ROS_WARN("reset when rosbag play back");
    p_imu->reset();
    flag_reset = false;
    return false;
  }

  ROS_INFO("[correction-stm] scan index: %d", scanIdx);
  time_solve = 0;

  state_propagated = state;

  if (is_first_frame) {
    first_lidar_time = time_measurement;
    is_first_frame = false;
  }


  if (feats_undistort->empty() || (feats_undistort == NULL)) {
    p_imu->first_lidar_time = first_lidar_time;
    cout << "FAST-LIO not ready" << endl;
    return false;
  }

  esikf_initialized = (time_measurement - first_lidar_time) < 0.0f ? false : true;
  std::cout << "ekf init " << esikf_initialized << " init map " << map_initialized << std::endl;
  if (esikf_initialized && !map_initialized) {
    ROS_INFO("[correction-stm] initializing map");
    pcl::PointCloud<pcl::PointXYZ>::Ptr world_lidar(new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Quaterniond q(state.rot_end);
    transformLidar(state, p_imu, feats_undistort, world_lidar);
    std::vector<pointWithCov> pv_list;
    ROS_INFO("size cloud %d", world_lidar->size());
    for (size_t i = 0; i < world_lidar->size(); i++) {
      pointWithCov pv;
      pv.point << world_lidar->points[i].x, world_lidar->points[i].y,
          world_lidar->points[i].z;
      V3D point_this(feats_undistort->points[i].x,
                    feats_undistort->points[i].y,
                    feats_undistort->points[i].z);
      // if z=0, error will occur in calcBodyCov. To be solved
      if (point_this[2] == 0) {
        point_this[2] = 0.001;
      }
      M3D cov;
      calcBodyCov(point_this, ranging_cov, angle_cov, cov);

      point_this = p_imu->Lid_rot_to_IMU * point_this +  Lidar_offset_to_IMU;
      M3D point_crossmat;
      point_crossmat << SKEW_SYM_MATRX(point_this);
      cov = state.rot_end * cov * state.rot_end.transpose() +
            (-point_crossmat) * state.cov.block<3, 3>(0, 0) *
                (-point_crossmat).transpose() +
            state.cov.block<3, 3>(3, 3);
      pv.cov = cov;
      pv_list.push_back(pv);
      Eigen::Vector3d sigma_pv = pv.cov.diagonal();
      sigma_pv[0] = sqrt(sigma_pv[0]);
      sigma_pv[1] = sqrt(sigma_pv[1]);
      sigma_pv[2] = sqrt(sigma_pv[2]);
    }

    buildVoxelMap(pv_list, max_voxel_size, max_layer, layer_size,
                  max_points_size, max_points_size, min_eigen_value,
                  vx_map);
    ROS_INFO("[correction-stm] voxel map built");

    scanIdx++;
    
    if (publish_voxel_map) {
      pubVoxelMap(vx_map, publish_max_voxel_layer, pub_voxel_map);
    }
    
    map_initialized = true;
    return true;
  }

  // downsample the feature points in a scan 
  auto t_downsample_start = std::chrono::high_resolution_clock::now();

  feats_undistorted_copy->clear();
  for (size_t i = 0; i < feats_undistort->size(); i++) {
    Eigen::Vector3f p(feats_undistort->points[i].x, 
      feats_undistort->points[i].y,
      feats_undistort->points[i].z);
    pcl::PointXYZINormal pi;
    pi.x = p(0);
    pi.y = p(1);
    pi.z = p(2);
    feats_undistorted_copy->points.push_back(pi);
  }
  feats_undistorted_copy->is_dense = false;
  downSizeFilterSurf.setInputCloud(feats_undistorted_copy);
  downSizeFilterSurf.filter(*feats_downsampled_body);
  auto t_downsample_end = std::chrono::high_resolution_clock::now();
  std::cout << " feats size:" << feats_undistort->size()
            << ", down size:" << feats_downsampled_body->size() << std::endl;
  auto t_downsample = std::chrono::duration_cast<std::chrono::duration<double>>
    (t_downsample_end - t_downsample_start) .count() * 1000;

  sort(feats_downsampled_body->points.begin(), feats_downsampled_body->points.end(), time_list);

  int rematch_num = 0;
  bool nearest_search_en = true;
  double total_residual;

  scan_match_time = 0.0;

  std::vector<M3D> body_var;
  std::vector<M3D> crossmat_list;

  // iterated state estimation 
  auto calc_point_cov_start = std::chrono::high_resolution_clock::now();
  for (size_t i = 0; i < feats_downsampled_body->size(); i++) {
    V3D point_this(feats_downsampled_body->points[i].x,
                  feats_downsampled_body->points[i].y,
                  feats_downsampled_body->points[i].z);
    if (point_this[2] == 0) {
      point_this[2] = 0.001;
    }
    M3D cov;
    calcBodyCov(point_this, ranging_cov, angle_cov, cov);
    point_this = p_imu->Lid_rot_to_IMU * point_this +  Lidar_offset_to_IMU;
    M3D point_crossmat;
    point_crossmat << SKEW_SYM_MATRX(point_this);
    crossmat_list.push_back(point_crossmat);
    M3D rot_var = state.cov.block<3, 3>(0, 0);
    M3D t_var = state.cov.block<3, 3>(3, 3);
    body_var.push_back(cov);
  }
  auto calc_point_cov_end = std::chrono::high_resolution_clock::now();
  double calc_point_cov_time = 
    std::chrono::duration_cast<std::chrono::duration<double>>(
      calc_point_cov_end - calc_point_cov_start).count() * 1000;
  
  // iterated state correction
  for (int iterCount = 0; iterCount < max_iterations_esikf; iterCount++) {
    laserCloudOri->clear();
    laserCloudNoeffect->clear();
    corr_normvect->clear();
    total_residual = 0.0;

    std::vector<double> r_list;
    std::vector<ptpl> ptpl_list;

    // LiDAR match based on 3 sigma criterion 
    vector<pointWithCov> pv_list;
    std::vector<M3D> var_list;
    pcl::PointCloud<pcl::PointXYZ>::Ptr world_lidar(new pcl::PointCloud<pcl::PointXYZ>);
    transformLidar(state, p_imu, feats_downsampled_body, world_lidar);
    for (size_t i = 0; i < feats_downsampled_body->size(); i++) {
      pointWithCov pv;
      pv.point << feats_downsampled_body->points[i].x,
          feats_downsampled_body->points[i].y, feats_downsampled_body->points[i].z;
      pv.point_world << world_lidar->points[i].x, world_lidar->points[i].y,
          world_lidar->points[i].z;
      M3D cov = body_var[i];
      M3D point_crossmat = crossmat_list[i];
      M3D rot_var = state.cov.block<3, 3>(0, 0);
      M3D t_var = state.cov.block<3, 3>(3, 3);
      cov = state.rot_end * cov * state.rot_end.transpose() +
            (-point_crossmat) * rot_var * (-point_crossmat.transpose()) +
            t_var;
      pv.cov = cov;
      pv_list.push_back(pv);
      var_list.push_back(cov);
    }
    auto scan_match_time_start = std::chrono::high_resolution_clock::now();
    std::vector<V3D> non_match_list;
    BuildResidualListOMP(vx_map, max_voxel_size, 3.0, max_layer, pv_list,
                        ptpl_list, non_match_list);

    auto scan_match_time_end = std::chrono::high_resolution_clock::now();

    effct_feat_num = 0;
    for (int i = 0; i < ptpl_list.size(); i++) {
      PointType pi_body;
      PointType pi_world;
      PointType pl;
      pi_body.x = ptpl_list[i].point(0);
      pi_body.y = ptpl_list[i].point(1);
      pi_body.z = ptpl_list[i].point(2);
      p_transformer->pointBodyToWorld(&pi_body, &pi_world, state);
      pl.x = ptpl_list[i].normal(0);
      pl.y = ptpl_list[i].normal(1);
      pl.z = ptpl_list[i].normal(2);
      effct_feat_num++;
      float dis = (pi_world.x * pl.x + pi_world.y * pl.y +
                  pi_world.z * pl.z + ptpl_list[i].d);
      // pl.intensity = dis;
      r_list.push_back(dis);
      laserCloudOri->push_back(pi_body);
      corr_normvect->push_back(pl);
      total_residual += fabs(dis);
    }
    res_mean_last = total_residual / effct_feat_num;

    ROS_INFO("scan id %d, features %d, mean residual: %f", scanIdx, effct_feat_num, res_mean_last);
    scan_match_time +=
        std::chrono::duration_cast<std::chrono::duration<double>>
        (scan_match_time_end - scan_match_time_start).count() * 1000;


    auto t_solve_start = std::chrono::high_resolution_clock::now();

    // Computation of measurement Jacobian matrix H and measurements vector
    MatrixXd Hsub(effct_feat_num, 6); // n_feat x 6, because measurements can only correct rotation (expressed as R^3 tangent plane pertubation) and position errors (R^3)
    MatrixXd Hsub_T_R_inv(6, effct_feat_num);
    VectorXd R_inv(effct_feat_num);
    VectorXd meas_vec(effct_feat_num);

    for (int i = 0; i < effct_feat_num; i++) {
      const PointType &laser_p = laserCloudOri->points[i];
      V3D point_this(laser_p.x, laser_p.y, laser_p.z);
      M3D cov;
      
      calcBodyCov(point_this, ranging_cov, angle_cov, cov);
      point_this = p_imu->Lid_rot_to_IMU * point_this + p_imu->Lid_offset_to_IMU; // CH check
      

      cov = state.rot_end * cov * state.rot_end.transpose();
      M3D point_crossmat;
      point_crossmat << SKEW_SYM_MATRX(point_this);
      const PointType &norm_p = corr_normvect->points[i];
      V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);
      V3D point_world = state.rot_end * point_this + state.pos_end;
      // get the normal vector of closest surface/corner 
      Eigen::Matrix<double, 1, 6> J_nq;
      J_nq.block<1, 3>(0, 0) = point_world - ptpl_list[i].center;
      J_nq.block<1, 3>(0, 3) = -ptpl_list[i].normal;
      double sigma_l = J_nq * ptpl_list[i].plane_cov * J_nq.transpose();
      R_inv(i) = 1.0 / (sigma_l + norm_vec.transpose() * cov * norm_vec);
      double ranging_dis = point_this.norm();
      // laserCloudOri->points[i].intensity = sqrt(R_inv(i));
      laserCloudOri->points[i].normal_x = r_list[i];
      laserCloudOri->points[i].normal_y = sqrt(sigma_l);
      laserCloudOri->points[i].normal_z =
          sqrt(norm_vec.transpose() * cov * norm_vec);
      laserCloudOri->points[i].curvature =
          sqrt(sigma_l + norm_vec.transpose() * cov * norm_vec);

      // calculate the Measuremnt Jacobian matrix H 
      V3D A(point_crossmat * state.rot_end.transpose() * norm_vec);
      Hsub.row(i) << VEC_FROM_ARRAY(A), norm_p.x, norm_p.y, norm_p.z;
      Hsub_T_R_inv.col(i) << A[0] * R_inv(i), A[1] * R_inv(i),
          A[2] * R_inv(i), norm_p.x * R_inv(i), norm_p.y * R_inv(i),
          norm_p.z * R_inv(i);
      // Measuremnt: distance to the closest surface/corner 
      meas_vec(i) = - r_list[i]; //norm_p.intensity;
    }
    MatrixXd K(DIM_STATE, effct_feat_num);

    EKF_stop_flg = false;
    flg_EKF_converged = false;

    // Iterative Kalman Filter Update 
    if (!esikf_initialized) {
      cout << "||||||||||Initiallizing LiDar||||||||||" << endl;
      //  only run in initialization period 
      MatrixXd H_init(MD(9, DIM_STATE)::Zero());
      MatrixXd z_init(VD(9)::Zero());
      H_init.block<3, 3>(0, 0) = M3D::Identity();
      H_init.block<3, 3>(3, 3) = M3D::Identity();
      H_init.block<3, 3>(6, 15) = M3D::Identity();
      z_init.block<3, 1>(0, 0) = -Log(state.rot_end);
      z_init.block<3, 1>(0, 0) = -state.pos_end;

      auto H_init_T = H_init.transpose();
      auto &&K_init =
          state.cov * H_init_T *
          (H_init * state.cov * H_init_T + 0.0001 * MD(9, 9)::Identity())
              .inverse();
      solution = K_init * z_init;

      state.resetpose();
      EKF_stop_flg = true;
    } else {
      auto &&Hsub_T = Hsub.transpose();
      H_T_H.block<6, 6>(0, 0) = Hsub_T_R_inv * Hsub;
      MD(DIM_STATE, DIM_STATE) &&K_1 =
          (H_T_H + (state.cov).inverse()).inverse();
      K = K_1.block<DIM_STATE, 6>(0, 0) * Hsub_T_R_inv;
      auto vec = state_propagated - state;

      // check for degeneration as in ICRA2016

    solution = K * meas_vec + vec - K * Hsub * vec.block<6, 1>(0, 0);
      if(enable_degeneracy_handling) {
        ROS_INFO("[correction-stm] degeneracy check");

      p_degeneracy_handler->update(time_measurement,
                                 iterCount,
                                 H_T_H.block<6, 6>(0, 0),
                                 solution);
      }

    }


      ROS_INFO("[correction-stm] before state update");
      ROS_INFO("[correction-stm] state: position: %.10f, %.10f, %.10f", state.pos_end[0], state.pos_end[1], state.pos_end[2]);
      ROS_INFO("[correction-stm] state: velocity: %.10f, %.10f, %.10f", state.vel_end[0], state.vel_end[1], state.vel_end[2]);
      state += solution;

      rot_add = solution.block<3, 1>(0, 0);
      t_add = solution.block<3, 1>(3, 0);

      ROS_INFO("[correction-stm] position changed by: %.4f", t_add.norm());

      if ((rot_add.norm() * 57.3 < 0.01) && (t_add.norm() * 100 < 0.015)) {
        flg_EKF_converged = true;
      }

      ROS_INFO("[correction-stm] after state update");
      ROS_INFO("[correction-stm] state: position: %.10f, %.10f, %.10f", state.pos_end[0], state.pos_end[1], state.pos_end[2]);
      ROS_INFO("[correction-stm] state: velocity: %.10f, %.10f, %.10f", state.vel_end[0], state.vel_end[1], state.vel_end[2]);

    
      euler_cur = RotMtoEuler(state.rot_end);
      // Rematch Judgement
      nearest_search_en = false;
      if (flg_EKF_converged || ((rematch_num == 0) && (iterCount == (max_iterations_esikf - 2)))) {
        nearest_search_en = true;
        rematch_num++;
      }

      // Convergence Judgements and Covariance Update
      if (!EKF_stop_flg &&
          (rematch_num >= 2 || (iterCount == max_iterations_esikf - 1))) {
        if (esikf_initialized) {
          // Covariance Update 
          G.setZero();
          G.block<DIM_STATE, 6>(0, 0) = K * Hsub;
          state.cov = (I_STATE - G) * state.cov;
          total_distance += (state.pos_end - position_last).norm();
          position_last = state.pos_end;

          geoQuat = tf::createQuaternionMsgFromRollPitchYaw(
              euler_cur(0), euler_cur(1), euler_cur(2));
        }
        EKF_stop_flg = true;
      }
      auto t_solve_end = std::chrono::high_resolution_clock::now();
      time_solve += std::chrono::duration_cast<std::chrono::duration<double>>(
                        t_solve_end - t_solve_start)
                        .count() *
                    1000;

      if (EKF_stop_flg)
        break;
    }

    // add the  points to the voxel map 
    auto map_incremental_start = std::chrono::high_resolution_clock::now();
    pcl::PointCloud<pcl::PointXYZ>::Ptr world_lidar(
        new pcl::PointCloud<pcl::PointXYZ>);
    transformLidar(state, p_imu, feats_downsampled_body, world_lidar);
    std::vector<pointWithCov> pv_list;
    for(size_t i = 0; i < world_lidar->size(); i++) {
      pointWithCov pv;
      pv.point << world_lidar->points[i].x, world_lidar->points[i].y,
          world_lidar->points[i].z;
      M3D point_crossmat = crossmat_list[i];
      M3D cov = body_var[i];
      cov = state.rot_end * cov * state.rot_end.transpose() +
            (-point_crossmat) * state.cov.block<3, 3>(0, 0) *
                (-point_crossmat).transpose() +
            state.cov.block<3, 3>(3, 3);
      pv.cov = cov;
      pv_list.push_back(pv);
    }
    std::sort(pv_list.begin(), pv_list.end(), var_contrast);
    updateVoxelMap(pv_list, max_voxel_size, max_layer, layer_size,
                  max_points_size, max_points_size, min_eigen_value,
                  vx_map);
    auto map_incremental_end = std::chrono::high_resolution_clock::now();
    map_incremental_time =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            map_incremental_end - map_incremental_start)
            .count() *
        1000; 
    ROS_INFO("[correction-stm] succesfully corrected pose");
    return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "vilain");
  ros::NodeHandle nh("~");
  ros::Rate rate(5000);
  bool status = ros::ok();

  ROS_INFO("[vilain] starting vilain node components"); 
  p_imu = make_shared<ImuProcessing>();
  p_lidar = make_shared<LidarProcessing>();
  p_vio = make_shared<VioProcessing>();
  p_transformer = make_shared<Transformer>();
  p_publisher = make_shared<VILAINPublisher>(nh, p_transformer);

  // degeneracy handler
  p_degeneracy_handler = make_shared<DegeneracyHandler>();
  p_degeneracy_handler->setup(20.0);

  ROS_INFO("[vilain] reading parameters"); 
  readParameters(nh);
  p_imu->set_extrinsic(t_D_to_IMU, R_D_to_IMU);
  p_transformer->set_R_D_to_IMU(R_D_to_IMU);
  p_transformer->set_t_D_to_IMU(t_D_to_IMU);
  p_imu->set_gyr_cov_scale(V3D(gyr_cov_scale, gyr_cov_scale, gyr_cov_scale));
  p_imu->set_acc_cov_scale(V3D(acc_cov_scale, acc_cov_scale, acc_cov_scale));
  p_imu->set_gyr_bias_cov(V3D(0.00001, 0.00001, 0.00001));
  p_imu->set_acc_bias_cov(V3D(0.00001, 0.00001, 0.00001));


  ROS_INFO("[vilain] enable imu propagation: %d", enable_imu_propagation);
  ROS_INFO("[vilain] enable vio correction:  %d", enable_vio_correction);
  ROS_INFO("[vilain] enable stm correction:  %d", enable_scan_to_map_correction); 


  // to be refactored
  pub_voxel_map = nh.advertise<visualization_msgs::MarkerArray>("/planes", 10000);
  downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
  G.setZero();
  H_T_H.setZero();
  I_STATE.setIdentity();

  // subscribers
  ros::Subscriber sub_imu;
  if (enable_imu_propagation) {
    ROS_INFO("[data] subscribing to imu data"); 
    sub_imu = nh.subscribe(topic_imu, 200000, callback_imu);
  }

  ROS_INFO("[data] subscribin g to point cloud data"); 
  ros::Subscriber sub_pcl = nh.subscribe(topic_lidar, 200000, callback_cloud);

  ROS_INFO("[data] starting devil synchronizer"); 
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_lidar(nh, topic_lidar, 100);
  message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> sub_pose(nh, topic_vins_pose, 100);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, geometry_msgs::PoseWithCovarianceStamped> DEVILSyncPolicy;
  message_filters::Synchronizer<DEVILSyncPolicy> sync(DEVILSyncPolicy(10), sub_lidar, sub_pose);
  sync.setMaxIntervalDuration(ros::Duration(0.1f / 1000)); // max package of 0.1ms
  sync.registerCallback(boost::bind(&callback_cloud_and_pose, _1, _2));

  bool map_needs_update = false;
  // main loop
  while (status) {
    if (flag_exit) break;
    ros::spinOnce();

    // vio not initialized yet or not used, using IMU-LiDAR only 
    if(!enable_vio_correction || !vio_initialized) {
      if(!buffer_lidar.empty()) {
        ROS_INFO("[correction-vio] VIO initialized %d", vio_initialized);
        // ROS_INFO("[correction-stm] VIO not initialized but will use LiDAR");

        // TO DO: correction-stm
        // ROS_INFO("[propagation]     to do");
        auto time_global_start = std::chrono::high_resolution_clock::now();
        if(propagate(time_buffer_lidar.front(), state)) {
          ROS_INFO("[propagation]     done");
          ROS_INFO("[correction-stm]  starting");
          
          // take point cloud
          p_lidar->undistort_cloud(buffer_lidar.front(), feats_undistorted);     
          std::cout << "state t" << state.pos_end << std::endl; 
          std::cout << "state R" << state.rot_end << std::endl; 

          // if we use lidar assistance
          if(enable_scan_to_map_correction)  { 
            if(correct_lidar(time_buffer_lidar.front(), state, feats_undistorted)) {
              ROS_INFO("[correction-stm]  successful");
              map_needs_update = true;
            }
            else {
              ROS_ERROR("[correction-stm]  not successful");
            }

          }
          else {
              ROS_INFO("[correction-stm]  skipped");
          }
          if(!enable_scan_to_map_correction) {
            // this means we're testing IMU-only mode, dead reckoning
            map_needs_update = true;
          }
          time_last_correction = time_buffer_lidar.front();
          buffer_lidar.pop_front();
          time_buffer_lidar.pop_front();

        auto time_global_end = std::chrono::high_resolution_clock::now();
        time_global_delta = std::chrono::duration_cast<std::chrono::duration<double>>(time_global_end - time_global_start).count()* 1000;
        std::cout << "###time_global_delta_novio### " << time_global_delta << std::endl;
        }
        else {
          // ROS_INFO("[propagation]     not yet");
          continue;
        }
      }
    }
    // this means vio has initialized
    if(enable_vio_correction && !buffer_sync.empty()) {
      ROS_INFO("[correction-vio] VIO initialized %d", vio_initialized);
      if(!vio_initialized) {
        ROS_INFO("[correction-vio] VIO initialized");
        vio_initialized = true;
      }

      // propagate state with imu or constant velocity model

      auto time_global_start = std::chrono::high_resolution_clock::now();

      ROS_INFO("[propagation]     to do");
      if(propagate(time_buffer_sync.front(), state)) {
        ROS_INFO("[propagation]     done");

        // take point cloud
        p_lidar->undistort_cloud(buffer_sync.front().first, feats_undistorted);        

        // if we use vio correction
        if(enable_vio_correction) {
          ROS_INFO("[correction-vio] will propagate state with VIO");
          ROS_INFO("[correction-vio]");

          p_vio->process_pose(buffer_sync.front().second, state);
          map_needs_update = true;
        }

        // if we use lidar assistance
        if(enable_scan_to_map_correction)  { 

          ROS_INFO("[correction-stm] will correct state with STM");
          ROS_INFO("[correction-stm]");
          
          if(correct_lidar(time_buffer_sync.front(), state, feats_undistorted)) {
            ROS_INFO("[correction-stm]  successful");
            map_needs_update = true;
          }
          else {
            ROS_ERROR("[correction-stm]  not successful");
          }
        }

        Eigen::Matrix4d pose_last_update;
        pose_last_update.block<3, 1>(0, 3) = state.pos_end;
        pose_last_update.block<3, 3>(0, 0) = state.rot_end;
        p_vio->pose_last_update = pose_last_update;

        // remove data from buffer and prepare next iterations
        time_last_correction = time_buffer_sync.front();
        buffer_sync.pop_front();  
        time_buffer_sync.pop_front();


        auto time_global_end = std::chrono::high_resolution_clock::now();
        time_global_delta = std::chrono::duration_cast<std::chrono::duration<double>>(time_global_end - time_global_start).count()* 1000;
        std::cout << "###time_global_delta### " << time_global_delta << std::endl;
      }
      else {
        // ROS_INFO("[propagation]     not yet");
        continue;
      }
    }

    // publish map
    if(enable_map_publication && map_needs_update) {
      // common to all correction methods
      ROS_INFO("[map] publishing point cloud");
      p_publisher->publish_odometry(state, time_last_correction);
      // publish_path(pub_path);

      if (publish_voxel_map) {
        pubVoxelMap(vx_map, publish_max_voxel_layer, pub_voxel_map);
      }
      
      ROS_INFO("[map] publishing %ld points", feats_undistorted->size());
      p_publisher->publish_cloud(pub_point_cloud_skip, feats_undistorted, state);
      
      ROS_INFO("[map] publishing downsampled cloud: %ld points", feats_downsampled_body->size());
      p_publisher->publish_downsampled_current_cloud(1, feats_downsampled_body, state);
      
      ROS_INFO("[map] publishing point cloud - done");
      map_needs_update = false;
    }
      
    status = ros::ok();
    rate.sleep();
  }
  return 0;
}
