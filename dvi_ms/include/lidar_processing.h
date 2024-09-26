#pragma once

#include "state.h"
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std;

#define IS_VALID(a) ((abs(a) > 1e8) ? true : false)

namespace l515_ros {
  struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;
//    float intensity;
    float rgb;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
} // namespace l515_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(
  l515_ros::Point,
  (float, x, x)(float, y, y)(float, z, z)
  (float, rgb, rgb)
)
// (float, intensity, intensity)
 
class LidarProcessing
{
  public:
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LidarProcessing();
  ~LidarProcessing();
  
  void undistort_cloud(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudType::Ptr &pcl_out);

  // sensor_msgs::PointCloud2::ConstPtr pointcloud;
  PointCloudXYZI pl_full, pl_corn;
  PointCloudType pl_surf;
  ros::Publisher pub_full, pub_surf, pub_corn;  
};
