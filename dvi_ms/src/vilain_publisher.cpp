#include "vilain_publisher.h"

VILAINPublisher::VILAINPublisher(ros::NodeHandle &n, std::shared_ptr<Transformer> _transformer) {
    transformer = _transformer;

    // set up publishers
    pub_cloud_full = n.advertise<sensor_msgs::PointCloud2>("/vilain_cloud", 100);
    pub_cloud_downsampled_current = n.advertise<sensor_msgs::PointCloud2>("/vilain_cloud_downsampled_current", 100);
    pub_odometry = nh.advertise<nav_msgs::Odometry>("/vilain_odometry", 10);
    // pub_voxel_map = nh.advertise<visualization_msgs::MarkerArray>("/planes", 10000)
}

VILAINPublisher::~VILAINPublisher() {}

void VILAINPublisher::set_ros_publishers(
    ros::Publisher & _pub_path_state_propagation,
    ros::Publisher & _pub_rgb_features,
    ros::Publisher & _pub_depth_alignment) {
    pub_path_state_propagation = _pub_path_state_propagation;
    pub_rgb_features = _pub_rgb_features;
    pub_depth_alignment = _pub_depth_alignment;
    ROS_INFO("[publisher] init topic: %s", pub_path_state_propagation.getTopic());
    ROS_INFO("[publisher] init topic: %s", pub_rgb_features.getTopic());
    ROS_INFO("[publisher] init topic: %s", pub_depth_alignment.getTopic());
}

void VILAINPublisher::publish_rgb_features(const sensor_msgs::Image::Ptr &img) {
    pub_rgb_features.publish(img);
}

void VILAINPublisher::publish_depth_alignment(const sensor_msgs::Image::Ptr &img) {
    pub_depth_alignment.publish(img);
}

void VILAINPublisher::publish_state_propagation(vector<V3D> poses_imu, vector<M3D> rotations_imu) {
    path_state_propagation.poses.clear();
    path_state_propagation.header.stamp = ros::Time::now();
    path_state_propagation.header.frame_id = "camera_init";
    for(int i = 0; i < poses_imu.size(); i++) {
        geometry_msgs::PoseStamped msg_pose_state_propagation;
        /*
            rot_kp.pos[i] = p(i);
            for (int j = 0; j < 3; j++)
            rot_kp.rot[i * 3 + j] = R(i, j);
        */
        msg_pose_state_propagation.pose.position.x = poses_imu[i][0]; 
        msg_pose_state_propagation.pose.position.y = poses_imu[i][1];
        msg_pose_state_propagation.pose.position.z = poses_imu[i][2];

        V3D euler_cur = RotMtoEuler(rotations_imu[i]);
        geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(euler_cur(0), euler_cur(1), euler_cur(2));
        msg_pose_state_propagation.pose.orientation = geoQuat;
        msg_pose_state_propagation.header.stamp = ros::Time::now();
        msg_pose_state_propagation.header.frame_id = "camera_init";
        path_state_propagation.poses.push_back(msg_pose_state_propagation);
    }
    pub_path_state_propagation.publish(path_state_propagation);
}


void VILAINPublisher::publish_odometry(StatesGroup& state, double time_update) {
    nav_msgs::Odometry msg;   
    msg.header.frame_id = "world";
    msg.header.stamp = ros::Time().fromSec(time_update);

    Eigen::Vector3d state_pose = state.pos_end;
    Eigen::Matrix3d state_rot = state.rot_end;

    if(publish_in_vins_frame) {
        state_pose = VINStoVILAIN.transpose() * state_pose;
        state_rot = VINStoVILAIN.transpose() * state_rot;
    }
    
    msg.pose.pose.position.x = state_pose(0);
    msg.pose.pose.position.y = state_pose(1);
    msg.pose.pose.position.z = state_pose(2);

    Eigen::Quaterniond pose_quat(state_rot);
    msg.pose.pose.orientation.x = pose_quat.x();
    msg.pose.pose.orientation.y = pose_quat.y();
    msg.pose.pose.orientation.z = pose_quat.z();
    msg.pose.pose.orientation.w = pose_quat.w();

    pub_odometry.publish(msg);
}

void pubPoseWithCovarianceStamped(StatesGroup& state, double time_update)
{
    /*
    Vector3d P = estimator.Ps[i] + estimator.Rs[i] * estimator.tic[0];
    Quaterniond R = Quaterniond(estimator.Rs[i] * estimator.ric[0]);

    geometry_msgs::PoseWithCovarianceStamped pose_with_cov_stamped;
    pose_with_cov_stamped.header = header;
    pose_with_cov_stamped.header.frame_id = "world";
    pose_with_cov_stamped.pose.pose.position.x = P.x();
    pose_with_cov_stamped.pose.pose.position.y = P.y();
    pose_with_cov_stamped.pose.pose.position.z = P.z();

    pose_with_cov_stamped.pose.pose.orientation.x = R.x();
    pose_with_cov_stamped.pose.pose.orientation.y = R.y();
    pose_with_cov_stamped.pose.pose.orientation.z = R.z();
    pose_with_cov_stamped.pose.pose.orientation.w = R.w();

    Eigen::MatrixXd cov(6, 6);
    cov.setIdentity();
    for(int i = 0; i < 36; i++) {
        pose_with_cov_stamped.pose.covariance[i] = cov.array()(i);
    }
    
    pub_pose_covariance_stamped.publish(pose_with_cov_stamped);*/
    
}

void VILAINPublisher::publish_cloud(
    const int point_skip,
    PointCloudType::Ptr& feats_undistort,
    StatesGroup& state)
{
    // CH PointCloudXYZI::Ptr laserCloudFullRes(dense_map_en ? feats_undistort : feats_down_body);
    PointCloudType::Ptr laserCloudFullRes(feats_undistort);
    int size = laserCloudFullRes->points.size();
    
    PointCloudType::Ptr laserCloudWorld(new PointCloudType(size, 1));
    for (int i = 0; i < size; i++) {
        transformer->RGBpointBodyToWorld(&laserCloudFullRes->points[i], &laserCloudWorld->points[i], state);
    }

    PointCloudType::Ptr laserCloudWorldPub(new PointCloudType);
    for (int i = 0; i < size; i += point_skip) {
        if(publish_in_vins_frame) {
            Eigen::Vector3d pt(laserCloudWorld->points[i].x, laserCloudWorld->points[i].y, laserCloudWorld->points[i].z);
            PointType2 pt_p = laserCloudWorld->points[i];
            pt = VINStoVILAIN.transpose() * pt;
            laserCloudWorld->points[i].x = pt.x();
            laserCloudWorld->points[i].y = pt.y();
            laserCloudWorld->points[i].z = pt.z();
            laserCloudWorldPub->points.push_back(laserCloudWorld->points[i]);
        }
        else {
            laserCloudWorldPub->points.push_back(laserCloudWorld->points[i]);
        }
    }
    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudWorldPub, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time::now(); 
    laserCloudmsg.header.frame_id = "world";
    pub_cloud_full.publish(laserCloudmsg);
}

void VILAINPublisher::publish_downsampled_current_cloud(
    const int point_skip,
    pcl::shared_ptr<PointCloudXYZI>& feats_downsampled_body,
    StatesGroup& state)
{
    pcl::shared_ptr<PointCloudXYZI> laserCloudFullRes(feats_downsampled_body);
    int size = laserCloudFullRes->points.size();
    
    PointCloudType::Ptr laserCloudWorld(new PointCloudType(size, 1));
    for (int i = 0; i < size; i++) {
        transformer->RGBpointBodyToWorld(&laserCloudFullRes->points[i], &laserCloudWorld->points[i], state);
    }

    PointCloudType::Ptr laserCloudWorldPub(new PointCloudType);
    for (int i = 0; i < size; i += point_skip) {
        if(publish_in_vins_frame) {
            Eigen::Vector3d pt(laserCloudWorld->points[i].x, laserCloudWorld->points[i].y, laserCloudWorld->points[i].z);
            PointType2 pt_p = laserCloudWorld->points[i];
            pt = VINStoVILAIN.transpose() * pt;
            laserCloudWorld->points[i].x = pt.x();
            laserCloudWorld->points[i].y = pt.y();
            laserCloudWorld->points[i].z = pt.z();
            laserCloudWorldPub->points.push_back(laserCloudWorld->points[i]);
        }
        else {
            laserCloudWorldPub->points.push_back(laserCloudWorld->points[i]);
        }
    }
    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudWorldPub, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time::now(); 
    laserCloudmsg.header.frame_id = "world";
    pub_cloud_downsampled_current.publish(laserCloudmsg);
}



void VILAINPublisher::publish_effect(
    const ros::Publisher &pubLaserCloudEffect,
    StatesGroup& state,
    PointCloudXYZI::Ptr &laserCloudOri,
    int effct_feat_num) 
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr effect_cloud_world(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(effct_feat_num, 1));
    for (int i = 0; i < effct_feat_num; i++) {
    transformer->RGBpointBodyToWorld(&laserCloudOri->points[i], &laserCloudWorld->points[i], state);
    pcl::PointXYZRGB pi;
    pi.x = laserCloudWorld->points[i].x;
    pi.y = laserCloudWorld->points[i].y;
    pi.z = laserCloudWorld->points[i].z;
    float v = 0.0; // laserCloudWorld->points[i].intensity / 100;
    v = 1.0 - v;
    uint8_t r, g, b;
    mapJet(v, 0, 1, r, g, b);
    pi.r = r;
    pi.g = g;
    pi.b = b;
    effect_cloud_world->points.push_back(pi);
    }

    sensor_msgs::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp =
        ros::Time::now(); //.fromSec(last_timestamp_lidar);
    laserCloudFullRes3.header.frame_id = "camera_init";
    pubLaserCloudEffect.publish(laserCloudFullRes3);
}