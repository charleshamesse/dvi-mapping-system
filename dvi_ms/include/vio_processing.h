#pragma once 

#include <ros/ros.h>
#include <stdio.h>
#include <deque>
#include <tuple>
#include <Eigen/Eigen>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "state.h"

class VioProcessing
{
  public:
    VioProcessing();

    // interface
    void process_pose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &, StatesGroup &);
    Eigen::Matrix4d pose_last_update;

    // internal
    std::deque<std::tuple<double, Eigen::Matrix4d, Eigen::Matrix<double, 6, 6>>> poses_vio;
    
    Eigen::Matrix3d R_vins_to_vilain;

};