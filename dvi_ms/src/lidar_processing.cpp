#include "lidar_processing.h"

LidarProcessing::LidarProcessing() {
}

LidarProcessing::~LidarProcessing() {}

void LidarProcessing::undistort_cloud(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudType::Ptr &pcl_out) {
  pl_surf.clear();
  pcl::PointCloud<l515_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  int plsize = pl_orig.points.size();
  for (int i = 0; i < pl_orig.size(); i++) {
    PointType2 added_pt;
    added_pt.x = pl_orig.points[i].x;
    added_pt.y = pl_orig.points[i].y;
    added_pt.z = pl_orig.points[i].z;
    // [CH] can use intensity or curvature to encode uncertainty
    // added_pt.intensity = pl_orig.points[i].intensity;

    // unpack rgb into r/g/b - or b/g/r in this case
    std::uint32_t rgb = *reinterpret_cast<int*>(&pl_orig.points[i].rgb);
    added_pt.r = (rgb >> 16) & 0x0000ff;
    added_pt.g = (rgb >> 8)  & 0x0000ff;
    added_pt.b = (rgb)       & 0x0000ff;
    pl_surf.push_back(added_pt);
  }
  *pcl_out = pl_surf;
}
