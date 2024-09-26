#pragma once

#include "common_lib.h"
#include "transformer.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <unordered_map>

class VILAINPublisher {
public:
  

    VILAINPublisher(ros::NodeHandle &, shared_ptr<Transformer>);
    ~VILAINPublisher();

    shared_ptr<Transformer> transformer;
    ros::NodeHandle nh;
    ros::Publisher pub_path_state_propagation; 
    ros::Publisher pub_path_posterior; 
    ros::Publisher pub_rgb_features; 
    ros::Publisher pub_depth_alignment; 
    ros::Publisher pub_cloud_full;
    ros::Publisher pub_cloud_downsampled_current;
    ros::Publisher pub_odometry;
    ros::Publisher pub_voxel_map;

    // coordinate frame change
    Eigen::Matrix3d VINStoVILAIN;
    bool publish_in_vins_frame = false;


    nav_msgs::Path path_state_propagation;
    geometry_msgs::PoseStamped msg_pose_state_propagation;

    void set_ros_publishers(ros::Publisher & _pub_path_state_propagation,
    ros::Publisher & _pub_rgb_features,
    ros::Publisher & _pub_depth_alignment);
    
    void publish_state_propagation(vector<V3D> poses_imu, vector<M3D> rotations_imu);

    void publish_cloud(
        const int point_skip,
        PointCloudType::Ptr& feats_undistort,
        StatesGroup& state);

    void publish_downsampled_current_cloud(
        const int point_skip,
        pcl::shared_ptr<PointCloudXYZI>& feats_downsampled_body,
        StatesGroup& state);


    void publish_odometry(StatesGroup& state, double time_update);

    void publish_effect(
        const ros::Publisher &pubLaserCloudEffect,
        StatesGroup& state,
        PointCloudXYZI::Ptr &laserCloudOri,
        int effct_feat_num);

    void publish_rgb_features(const sensor_msgs::Image::Ptr &img);
    void publish_depth_alignment(const sensor_msgs::Image::Ptr &img);

private:

};