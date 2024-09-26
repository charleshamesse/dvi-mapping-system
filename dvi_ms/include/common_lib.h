#pragma once

#include "state.h"
#include <Eigen/Eigen>
#include <eigen_conversions/eigen_msg.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/Imu.h>
#include <so3_math.h>
#include <tf/transform_broadcaster.h>
#include <dvi_ms/Pose6D.h>
#include <dvi_ms/States.h>

using namespace std;
using namespace Eigen;

#define PI_M (3.14159265358)
#define G_m_s2 (9.81)   // Gravaty const in GuangDong/China
#define DIM_STATE (18)  // Dimension of states (Let Dim(SO(3)) = 3)
#define INIT_COV (0.0000001)
#define VEC_FROM_ARRAY(v) v[0], v[1], v[2]

typedef dvi_ms::Pose6D Pose6D;
typedef pcl::PointXYZINormal PointType;
typedef pcl::PointXYZRGBNormal PointType2;
typedef pcl::PointCloud<PointType> PointCloudXYZI;
typedef pcl::PointCloud<PointType2> PointCloudType;


typedef vector<PointType, Eigen::aligned_allocator<PointType>> PointVector;
typedef Vector3d V3D;
typedef Matrix3d M3D;
typedef Vector3f V3F;
typedef Matrix3f M3F;

#define MD(a, b) Matrix<double, (a), (b)>
#define VD(a) Matrix<double, (a), 1>
#define MF(a, b) Matrix<float, (a), (b)>
#define VF(a) Matrix<float, (a), 1>

extern M3D Eye3d;
extern M3F Eye3f;
extern V3D Zero3d;
extern V3F Zero3f;
extern Vector3d Lidar_offset_to_IMU;

// measure struct
struct MeasureGroup // lidar data and imu dates for the curent process
{
  MeasureGroup(); 
  double lidar_beg_time;
  PointCloudType::Ptr lidar;
  deque<sensor_msgs::Imu::ConstPtr> imu;
};

// state struct
struct StatesGroup {
  M3D rot_end; // the estimated rotation matrix (world frame)
  V3D pos_end; // the estimated position (world frame)
  V3D vel_end; // the estimated velocity (world frame)
  V3D bias_g; // gyroscope bias
  V3D bias_a; // accelerator bias
  V3D gravity; // the estimated gravity acceleration
  Matrix<double, DIM_STATE, DIM_STATE> cov; // states covariance

  StatesGroup();
  StatesGroup(const StatesGroup &b);
  StatesGroup &operator=(const StatesGroup &b);
  StatesGroup operator+(const Matrix<double, DIM_STATE, 1> &state_add);
  StatesGroup &operator+=(const Matrix<double, DIM_STATE, 1> &state_add);
  Matrix<double, DIM_STATE, 1> operator-(const StatesGroup &b);
  void resetpose();
};

// utility methods
const bool time_list(PointType &x, PointType &y);

// utility template methods
template <typename T>
T rad2deg(T radians) { return radians * 180.0 / PI_M; }

template <typename T>
T deg2rad(T degrees) { return degrees * PI_M / 180.0; }

template <typename T>
auto set_pose6d(const double t, const Matrix<T, 3, 1> &a,
                const Matrix<T, 3, 1> &g, const Matrix<T, 3, 1> &v,
                const Matrix<T, 3, 1> &p, const Matrix<T, 3, 3> &R) {
  Pose6D rot_kp;
  rot_kp.offset_time = t;
  for (int i = 0; i < 3; i++) {
    rot_kp.acc[i] = a(i);
    rot_kp.gyr[i] = g(i);
    rot_kp.vel[i] = v(i);
    rot_kp.pos[i] = p(i);
    for (int j = 0; j < 3; j++)
      rot_kp.rot[i * 3 + j] = R(i, j);
  }
  return move(rot_kp);
}


void mapJet(double v, double vmin, double vmax, uint8_t &r, uint8_t &g, uint8_t &b);