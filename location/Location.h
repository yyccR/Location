//
// Created by yangcheng on 2019/1/14.
//


#include "../system/Status.h"
#include "Eigen/Dense"

#ifndef LOCATION_LOCATION_H
#define LOCATION_LOCATION_H

using namespace Eigen;

class Location {
public:

    void PredictCurrentPosition(Vector3d &gyro_data, Vector3d &acc_data, Vector3d &mag_data, Status *status, double t);

};


#endif //LOCATION_LOCATION_H
