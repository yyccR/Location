//
// Created by yangcheng on 2018/12/24.
//

#include "../system/Status.h"
#include "Eigen/Dense"

#ifndef LOCATION_MAGNETOMETER_H
#define LOCATION_MAGNETOMETER_H

using namespace Eigen;

class Magnetometer {
public:

    // 归一化.
    Vector3d Normalise(Vector3d &m) const;

    // 地磁感应误差计算
    Vector3d GetMagError(Matrix3d &b2n, Vector3d &originMag) const;

    Vector3d GetMagError(Vector4d &q, Vector3d &originMag);

    // 地磁计标定
    void MagCalibration(MatrixXd &input_data, Status *status);
};


#endif //LOCATION_MAGNETOMETER_H
