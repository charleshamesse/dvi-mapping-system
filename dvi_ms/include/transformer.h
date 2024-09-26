
#pragma once 

#include "common_lib.h"

class Transformer {
public:
  

  Transformer();
  ~Transformer();

  V3D t_D_to_IMU;
  M3D R_D_to_IMU;

  void set_t_D_to_IMU(const V3D &transl) {
    t_D_to_IMU = transl;
  }
  void set_R_D_to_IMU(const M3D &R) {
    R_D_to_IMU = R;
  }

// project the lidar scan to world frame
void pointBodyToWorld(PointType const *const pi, PointType *const po, StatesGroup& state) {
  V3D p_body(pi->x, pi->y, pi->z);
  p_body = R_D_to_IMU * p_body + t_D_to_IMU;
  V3D p_global(state.rot_end * (p_body) + state.pos_end);
  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  // po->intensity = pi->intensity;
}

template <typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po, StatesGroup& state) {
  V3D p_body(pi[0], pi[1], pi[2]);
  p_body = R_D_to_IMU * p_body + t_D_to_IMU;
  V3D p_global(state.rot_end * (p_body) + state.pos_end);
  po[0] = p_global(0);
  po[1] = p_global(1);
  po[2] = p_global(2);
}

void RGBpointBodyToWorld(PointType const *const pi, PointType2 *const po, StatesGroup& state) {
  V3D p_body(pi->x, pi->y, pi->z);
  p_body = R_D_to_IMU * p_body + t_D_to_IMU;
  V3D p_global(state.rot_end * (p_body) + state.pos_end);
  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
}

void RGBpointBodyToWorld(PointType const *const pi, PointType *const po, StatesGroup& state) {
  V3D p_body(pi->x, pi->y, pi->z);
  p_body = R_D_to_IMU * p_body + t_D_to_IMU;
  V3D p_global(state.rot_end * (p_body) + state.pos_end);
  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->curvature = pi->curvature;
  po->normal_x = pi->normal_x;
  po->normal_y = pi->normal_y;
  po->normal_z = pi->normal_z;

}

void RGBpointBodyToWorld(PointType2 const *const pi, PointType2 *const po, StatesGroup& state) {
  V3D p_body(pi->x, pi->y, pi->z);
  p_body = R_D_to_IMU * p_body + t_D_to_IMU;
  V3D p_global(state.rot_end * (p_body) + state.pos_end);
  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->curvature = pi->curvature;
  po->normal_x = pi->normal_x;
  po->normal_y = pi->normal_y;
  po->normal_z = pi->normal_z;

  po->r = pi->r;
  po->g = pi->g;
  po->b = pi->b;

}

template<typename T>
Eigen::Matrix<T, 3, 1> RotMtoEuler(const Eigen::Matrix<T, 3, 3> &rot)
{
    T sy = sqrt(rot(0,0)*rot(0,0) + rot(1,0)*rot(1,0));
    bool singular = sy < 1e-6;
    T x, y, z;
    if(!singular)
    {
        x = atan2(rot(2, 1), rot(2, 2));
        y = atan2(-rot(2, 0), sy);   
        z = atan2(rot(1, 0), rot(0, 0));  
    }
    else
    {    
        x = atan2(-rot(1, 2), rot(1, 1));    
        y = atan2(-rot(2, 0), sy);    
        z = 0;
    }
    Eigen::Matrix<T, 3, 1> ang(x, y, z);
    return ang;
}
private:

};