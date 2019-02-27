//
// Created by yangcheng on 2018/12/26.
//
#include "Eigen/Dense"

#ifndef LOCATION_ARHS_H
#define LOCATION_ARHS_H


class AHRS {
public:

    // 姿态更新.
    Eigen::Vector4d UpdateAttitude(Eigen::Vector3d *err, Eigen::Vector4d &q_attitude, Eigen::Vector3d &gyro,
                                   Eigen::Vector3d &acc, Eigen::Vector3d &mag,
                                   double &ki, double &kp, double &halfT) const ;

};


#endif //LOCATION_ARHS_H
