#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <ros/ros.h>

#define DIM_STATE 18  
#define DIM_EV 6

class DegeneracyHandler {
public:
    DegeneracyHandler();
    ~DegeneracyHandler();
    void setup(
            double t_init
    );
    void update(
            double ts,
            int it,
            const Eigen::Matrix<double, DIM_EV, DIM_EV> & HTH,
            Eigen::Matrix<double, DIM_STATE, 1> & solution);
private:
    double  t_init,
            ts_first_update;
    bool flag_first_update;
    Eigen::Matrix<double, DIM_EV, 1> ev_min;
    double ev_single_min;
    double ev_threshold_manual;
    bool mode_soft;
};