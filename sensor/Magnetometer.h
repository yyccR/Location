//
// Created by yangcheng on 2018/12/24.
//

#include "../system/Status.h"
#include "Eigen/Dense"

#ifndef LOCATION_MAGNETOMETER_H
#define LOCATION_MAGNETOMETER_H


class Magnetometer {
public:

    // 归一化.
    Eigen::Vector3d Normalise(Eigen::Vector3d &m) const;

    // 地磁感应误差计算
    Eigen::Vector3d GetMagError(Eigen::Matrix3d &b2n, Eigen::Vector3d &originMag) const;

    Eigen::Vector3d GetMagError(Eigen::Vector4d &q, Eigen::Vector3d &originMag);

    // 地磁计标定
    void MagCalibration(Eigen::MatrixXd &input_data, Status *status);
};


#endif //LOCATION_MAGNETOMETER_H
