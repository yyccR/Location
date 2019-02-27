//
// Created by yangcheng on 2019/1/14.
//


#include "../system/Status.h"
#include "Eigen/Dense"

#ifndef LOCATION_LOCATION_H
#define LOCATION_LOCATION_H

class Location {
public:

    void PredictCurrentPosition(Eigen::Vector3d &gyro_data, Eigen::Vector3d &acc_data, Eigen::Vector3d &mag_data,
                                Eigen::VectorXd &gps_data, Eigen::Vector3d &g_data, Eigen::Vector3d &ornt_data, Status *status);

};


#endif //LOCATION_LOCATION_H
