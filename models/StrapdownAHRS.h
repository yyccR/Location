//
// Created by yangcheng on 2019/3/13.
//

#ifndef LOCATION_STRAPDOWNAHRS_H
#define LOCATION_STRAPDOWNAHRS_H

#include "Eigen/Dense"
#include "../system/Status.h"


class StrapdownAHRS {
public:

    // 捷联式姿态更新.
    Eigen::Vector4d
    StrapdownUpdateAttitude(Eigen::Vector4d &q_attitude, Eigen::Vector3d &gyro, routing::Status *status);
};


#endif //LOCATION_STRAPDOWNAHRS_H
