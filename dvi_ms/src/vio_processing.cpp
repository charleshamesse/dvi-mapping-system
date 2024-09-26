#include "vio_processing.h"

VioProcessing::VioProcessing()
{
    ROS_INFO("[correction-vio] begins");
    R_vins_to_vilain << 
        1,  0,  0,
        0,  1,  0,
        0,  0,  1;
    /* L515 
    R_vins_to_vilain << 
        1,  0,  0,
        0,  0,  -1,
        0,  1,  0;*/
}

void VioProcessing::process_pose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg_pose, StatesGroup &state) {
    ROS_INFO("[correction-vio] processing pose");
    
    // 1. parse pose from VIO
    std::tuple<double, Eigen::Matrix4d, Eigen::Matrix<double, 6, 6>> pose_vio;


    // 1.1 stamp
    std::get<0>(pose_vio) = msg_pose->header.stamp.toSec(); // get returns ref to value
    
    // 1.2 pose R and t
    Eigen::Matrix4d pose_pose;
    pose_pose.setIdentity();
    pose_pose.block<3, 1>(0, 3) << Eigen::Vector3d(msg_pose->pose.pose.position.x, msg_pose->pose.pose.position.y, msg_pose->pose.pose.position.z);
    Eigen::Quaterniond pose_quat;
    tf::quaternionMsgToEigen(msg_pose->pose.pose.orientation, pose_quat);
    Eigen::Matrix3d pose_rot(pose_quat.toRotationMatrix());
    pose_pose.block<3, 3>(0, 0) << pose_rot;
    pose_pose.block<3, 1>(0, 3) = R_vins_to_vilain * pose_pose.block<3, 1>(0, 3);
    pose_pose.block<3, 3>(0, 0) = R_vins_to_vilain * pose_pose.block<3, 3>(0, 0);
    std::get<1>(pose_vio) = pose_pose;


    // 1.3 pose cov
    Eigen::Matrix<double, 6, 6> pose_cov(msg_pose->pose.covariance.data());
    // pose_cov = pose_cov * 1e-12; // CH to be removed
    std::get<2>(pose_vio) = pose_cov;

    ROS_INFO("using cov:");
    std::cout << pose_cov << std::endl;

    // if it's at least the second pose update from VIO
    if(poses_vio.size() > 0) {
        ROS_INFO("[correction-vio] previous pose present - will update state with relative pose");

        // 2. compute pose delta with previous pose
        Eigen::Matrix4d pose_delta;
        pose_delta =  std::get<1>(poses_vio.back()).inverse() * std::get<1>(pose_vio);
        
        /*
        std::cout << "poses_vio.back()" << std::endl;
        std::cout << std::get<1>(poses_vio.back()) << std::endl;

        std::cout << "pose_vio" << std::endl;
        std::cout << std::get<1>(pose_vio) << std::endl;
        
        std::cout << "pose delta" << std::endl;
        std::cout << pose_delta << std::endl;*/

        // 3. "measure" current state by pose delta
        Eigen::Matrix4d pose_updated;
        pose_updated = pose_last_update * pose_delta;
        
        // 4. filter
        VectorXd z(6);
        z.setZero();
        Eigen::Matrix3d R_vio = pose_updated.block<3, 3>(0, 0);
        Eigen::Vector3d t_vio = pose_updated.block<3, 1>(0, 3);
        Eigen::Matrix3d R_est;
        Eigen::Vector3d t_est;
        Eigen::Matrix3d R_delta;
        Eigen::Vector3d t_delta;
        Eigen::Vector3d r_delta;

        Eigen::MatrixXd I_state(DIM_STATE, DIM_STATE);
        I_state.setIdentity();
        Eigen::MatrixXd H_dx(6, DIM_STATE);
        Eigen::MatrixXd R_meas(6, 6);

        Eigen::MatrixXd gain(DIM_STATE, 6);
        Eigen::VectorXd solution(DIM_STATE);
        StatesGroup state_propagated = state;

        int esikf_vio_iterations = 5;
        // 4.1 compute compute residual
        R_est = state.rot_end;
        t_est = state.pos_end;
        R_delta = R_est.transpose() * R_vio;
        r_delta = Log(R_delta);
        t_delta = t_vio - t_est;
        std::cout << "t_delta: " << t_delta.norm() << std::endl;
        if(t_delta.norm() > 0.15) {
            ROS_ERROR("[correction-vio] pose delta too large, resetting");
            poses_vio.clear();
            poses_vio.push_back(pose_vio);
            return;
        }

        z << r_delta[0], r_delta[1], r_delta[2], t_delta[0], t_delta[1], t_delta[2];
        std::cout << "residual norm: " << z.norm() << std::endl;
        for(int it = 0; it < esikf_vio_iterations; it++) {

            // 4.2 compute Jacobian w.r.t. error-state
            H_dx.setIdentity();
            H_dx = - H_dx;

            // 4.3 compute Jacobian w.r.t. error-measurement
            /*
            R_meas.setIdentity();
            R_meas = 0.000005 * R_meas;*/

            // 4.4 compute Kalman gain and solution and apply
            gain << state.cov * H_dx.transpose() * (H_dx * state.cov * H_dx.transpose() + std::get<2>(pose_vio)).inverse();
            solution << -gain*z - (I_state - gain * H_dx) * (state - state_propagated); // ignoring \cal{H}

            ROS_INFO("[correction-vio] state before: %.10f, %.10f, %.10f", state.pos_end[0], state.pos_end[1], state.pos_end[2]);
            state += solution;
            ROS_INFO("[correction-vio] state after:  %.10f, %.10f, %.10f", state.pos_end[0], state.pos_end[1], state.pos_end[2]);

            // 4.5 compute residual again
            R_est = state.rot_end;
            t_est = state.pos_end;
            R_delta = R_est.transpose() * R_vio;
            r_delta = Log(R_delta);
            t_delta = t_vio - t_est;
            z << r_delta[0], r_delta[1], r_delta[2], t_delta[0], t_delta[1], t_delta[2];

            // std::cout << "residual:" << std::endl << z << std::endl;
            std::cout << "residual norm:" << std::endl << z.norm() << std::endl;
        }
        // 4.6 update covariance
        state.cov = (I_state - gain * H_dx) * state.cov;        
    }
    else {
        ROS_INFO("[correction-vio] no previous pose - doing nothing"); 
        /* forcing update to absolute VIO pose");
        state.rot_end = pose_pose.block<3, 3>(0, 0);
        state.pos_end = pose_pose.block<3, 1>(0, 3);
        std::cout << "state" << std::endl;
        std::cout << pose_pose << std::endl; */
    }
    // 4. add pose to list of poses
    poses_vio.push_back(pose_vio);


    

}
