#include "parameters.h"

bool    enable_vio_correction = true,
        enable_scan_to_map_correction = true,
        enable_imu_propagation = false,
        enable_degeneracy_handling = false,
        enable_map_publication = true;

std::string topic_lidar,
            topic_imu,
            topic_vins_pose;

double  degeneracy_min_eigen_value,
        ranging_cov,
        angle_cov,
        gyr_cov_scale,
        acc_cov_scale,
        max_voxel_size = 0.25,
        filter_size_surf_min = 0.5,
        min_eigen_value = 0.01;

bool    publish_voxel_map = true,
        publish_point_cloud = true,
        publish_max_voxel_layer = true;

int pub_point_cloud_skip = 1,
    max_iterations_esikf = 3,
    max_points_size = 200,
    max_layer = 2;

std::vector<int> layer_point_size = std::vector<int>();
std::vector<int> layer_size  = std::vector<int>();

Eigen::Matrix3d imu_R;
Eigen::Vector3d imu_t;

Eigen::Matrix3d R_D_to_IMU;
Eigen::Vector3d t_D_to_IMU;
Eigen::Matrix3d R_RGB_to_IMU;
Eigen::Vector3d t_RGB_to_IMU;

template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}

void readParameters(ros::NodeHandle &n) {
    std::string config_file;
    config_file = readParam<std::string>(n, "config_file");
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }
    std::string VINS_FOLDER_PATH = readParam<std::string>(n, "vins_folder");

    fsSettings["lidar_topic"] >> topic_lidar;
    fsSettings["imu_topic"] >> topic_imu;
    fsSettings["vins_pose_topic"] >> topic_vins_pose;

    cv::FileNode fn = fsSettings["vilain"];
    // state estimation params
    enable_imu_propagation = static_cast<int>(fn["imu_state_propagation"]);
    enable_scan_to_map_correction = static_cast<int>(fn["scan_to_map_state_correction"]);
    enable_vio_correction = static_cast<int>(fn["vio_state_correction"]);


    // degeneracy handling
    enable_degeneracy_handling = (int) fn["enable_degeneracy_handling"];
    degeneracy_min_eigen_value = (int) fn["degeneracy_handling_min_eigen_value"];

    // noise model params
    ranging_cov = fn["noise_model_ranging_cov"];
    angle_cov = fn["noise_model_angle_cov"];
    gyr_cov_scale = fn["noise_model_gyr_cov_scale"];
    acc_cov_scale = fn["noise_model_acc_cov_scale"];
    ROS_INFO("acc_cov_scale %f", acc_cov_scale);

    // IMU - camera params
    // enable_imu_propagation = (int) fn["imu_en"];
    // ROS_INFO("imu_en %d", enable_imu_propagation);
    cv::Mat cv_R, cv_T;
    fsSettings["extrinsicTranslation"] >> cv_T;
    fsSettings["extrinsicRotation"] >> cv_R;
    cv::cv2eigen(cv_R, imu_R);
    cv::cv2eigen(cv_T, imu_t);

    // Depth to IMU transformation, was "p_imu->Lid_rot_to_IMU"
    cv::Mat R_D_to_IMU_cv, t_D_to_IMU_cv;
    fsSettings["R_D_to_IMU"] >> R_D_to_IMU_cv;
    fsSettings["t_D_to_IMU"] >> t_D_to_IMU_cv;
    cv::cv2eigen(R_D_to_IMU_cv, R_D_to_IMU);
    cv::cv2eigen(t_D_to_IMU_cv, t_D_to_IMU);

    // RGB to IMU transformation
    cv::Mat R_RGB_to_IMU_cv, t_RGB_to_IMU_cv;
    fsSettings["R_RGB_to_IMU"] >> R_RGB_to_IMU_cv;
    fsSettings["t_RGB_to_IMU"] >> t_RGB_to_IMU_cv;
    cv::cv2eigen(R_RGB_to_IMU_cv, R_RGB_to_IMU);
    cv::cv2eigen(t_RGB_to_IMU_cv, t_RGB_to_IMU);
    
    // transformer->set_t_cloud_to_IMU(eigen_T);


    // visualization params
    ROS_INFO("vis params");
    publish_voxel_map = (int) fn["visualization_pub_voxel_map"];
    publish_point_cloud = (int) fn["visualization_pub_point_cloud"];
    publish_max_voxel_layer = (int) fn["visualization_publish_max_voxel_layer"];
    pub_point_cloud_skip = (int) fn["visualization_pub_point_cloud_skip"];

    // mapping algorithm params
    max_iterations_esikf = fn["mapping_max_iteration"];
    max_points_size = fn["mapping_max_points_size"];
     // TODO CH put as parameter
    layer_point_size.push_back(20);
    layer_point_size.push_back(10);
    layer_point_size.push_back(10);
    layer_point_size.push_back(5);
    layer_point_size.push_back(5);
    for (int i = 0; i < layer_point_size.size(); i++) {
      layer_size.push_back(layer_point_size[i]);
    }
    max_layer = fn["mapping_max_layer"];
    max_voxel_size = fn["mapping_voxel_size"];
    filter_size_surf_min = fn["mapping_down_sample_size"];
    min_eigen_value= fn["mapping_planar_threshold"];
}
