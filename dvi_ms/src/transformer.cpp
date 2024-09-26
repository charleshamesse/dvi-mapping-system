#include "transformer.h"

Transformer::Transformer() : 
    t_D_to_IMU(V3D(0, 0, 0)),
    R_D_to_IMU(M3D::Identity()) {

}

Transformer::~Transformer() {}