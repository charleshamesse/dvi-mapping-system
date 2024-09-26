#pragma once

#include <iostream>
#include <string> 
#include <fstream>
#include <ros/ros.h>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

extern bool enable_vio_correction,
            enable_scan_to_map_correction,
            enable_imu_propagation,
            enable_degeneracy_handling,
            enable_map_publication;

extern std::string  topic_lidar,
                    topic_imu,
                    topic_vins_pose;
                    
extern double   degeneracy_min_eigen_value,
                ranging_cov,
                angle_cov,
                gyr_cov_scale,
                acc_cov_scale,
                max_voxel_size,
                filter_size_surf_min,
                min_eigen_value;

extern bool    publish_voxel_map,
                publish_point_cloud,
                publish_max_voxel_layer;

extern int  pub_point_cloud_skip,
            max_iterations_esikf,
            max_points_size,
            max_layer;

extern std::vector<int> layer_size;

extern Eigen::Matrix3d imu_R;
extern Eigen::Vector3d imu_t;

extern Eigen::Matrix3d R_D_to_IMU;
extern Eigen::Vector3d t_D_to_IMU;

extern Eigen::Matrix3d R_RGB_to_IMU;
extern Eigen::Vector3d t_RGB_to_IMU;

extern bool publish_voxel_map;

void readParameters(ros::NodeHandle &n);