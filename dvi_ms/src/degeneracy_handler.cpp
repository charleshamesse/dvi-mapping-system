#include "degeneracy_handler.h"

DegeneracyHandler::DegeneracyHandler() {
    ROS_INFO("DegeneracyHandler::DegeneracyHandler");
    flag_first_update = true;
    ts_first_update = -1;
    ev_single_min = -1;
    ev_threshold_manual = 0; // 0.0 * 1000;
    mode_soft = true;
}
DegeneracyHandler::~DegeneracyHandler() {
    ROS_INFO("DegeneracyHandler::DegeneracyHandler");
}
void DegeneracyHandler::setup(double _t_init) {
    std::string str_mode = "soft";
    if(!mode_soft) str_mode = "hard";
    ROS_INFO("DegeneracyHandler::setup %f mode %s", ev_threshold_manual, str_mode.c_str());
    t_init = _t_init;
    // FILE *fp_ev;
    //  fp_ev = fopen(result_path_ev.c_str(), "w");
}
void DegeneracyHandler::update(
        double ts,
        int it,
        const Eigen::Matrix<double, DIM_EV, DIM_EV> & HTH,
        Eigen::Matrix<double, DIM_STATE, 1> & solution) {
    ROS_INFO("DegeneracyHandler::update");

    // in any case, solve eigen values and vectors
    Eigen::EigenSolver<Eigen::MatrixXd> eigensolver;
    eigensolver.compute(HTH);

    // note: column k of the returned matrix is an eigenvector
    // corresponding to eigenvalue number k as returned by eigenvalues(). T
    // we take the transpose to match the paper: in Vf each row is an eigenvector
    Eigen::MatrixXd Vf = eigensolver.eigenvectors().real().transpose(); // 6 x 6
    Eigen::VectorXd ev(DIM_EV);
    ev << eigensolver.eigenvalues().real();
    // ev << HTH.eigenvalues().real();

    // if first update, save timestamp
    if(flag_first_update) {
        ts_first_update = ts;
        ev_min << ev;
        ev_single_min = ev.coeff(0); // set with some value
        flag_first_update = false;
        return;
    }

    // if we haven't passed t_init yet, keep looking for the min
    if(ts - ts_first_update < t_init) {
        for(int ev_i = 0; ev_i < DIM_EV; ev_i++) {
            if(ev.coeff(ev_i) < ev_min.coeff(ev_i)) {
                ROS_INFO("ev_i %d, new min: %f", ev_i, ev.coeff(ev_i));
                ev_min(ev_i) = ev.coeff(ev_i);
            }
            if(ev.coeff(ev_i) < ev_single_min) {
                ROS_INFO("ev_i %d, new single min: %f", ev_i, ev.coeff(ev_i));
                ev_single_min = ev.coeff(ev_i);
            }
        }
        return;
    }

    // here we have accumulated sufficient data, we will now apply the remapping
    Eigen::MatrixXd Vp(DIM_EV, DIM_EV);
    Eigen::MatrixXd Vu(DIM_EV, DIM_EV);
    Vp << Vf;
    Vu << Vf;
    double thresh_effective = ev_single_min  / 100.;
    ROS_INFO("single min: %f", thresh_effective);
    for(int ev_i = 0; ev_i < DIM_EV; ev_i++) {
        double ratio = ev.coeff(ev_i) / thresh_effective;
        std::cout << "ev_i " << ev_i << ": " << ev.coeff(ev_i) << " ratio: " << ratio << std::endl;
        // if it detected as degenerate
        if(ev.coeff(ev_i) < thresh_effective) {
            std::cout << "ev_i " << ev_i << " << dis. " << ev.coeff(ev_i)
                      <<  " < " << thresh_effective  << "\t ratio: " << ratio << "\n";
            if(mode_soft) {
                // decrease the weight of the observation
                Vu.row(ev_i) *= ratio;
            }
            else {
                Vu.row(ev_i).setZero(); // *= ratio;
            }
        }
        else { // not detected
            Vp.row(ev_i).setZero();
        }
    }
    std::cout << std::endl;
    // fprintf(fp_ev, "%lf %d %lf %lf %lf %lf %lf %lf",last_timestamp_lidar, iterCount, ev_HTH[0] ,ev_HTH[1] ,ev_HTH[2] ,ev_HTH[3] , ev_HTH[4] , ev_HTH[5] );
    // fprintf(fp_ev, "\n");
    // fflush(fp_ev);

    std::cout << solution.transpose() << std::endl;
    solution.block<6, 6>(0 ,0) << Vf.inverse() * Vu * solution.block<6, 6>(0 ,0);
    std::cout << solution.transpose() << std::endl;
}