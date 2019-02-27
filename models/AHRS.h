//
// Created by yangcheng on 2018/12/26.
//
#include "Eigen/Dense"

#ifndef LOCATION_ARHS_H
#define LOCATION_ARHS_H

using namespace Eigen;

class AHRS {
public:

    // 姿态更新.
    Vector4d UpdateAttitude(Vector3d *err, Vector4d &q_attitude, Vector3d &gyro, Vector3d &acc, Vector3d &mag, double &ki, double &kp, double &halfT) const ;



};


#endif //LOCATION_ARHS_H
