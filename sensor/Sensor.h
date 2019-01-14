//
// Created by yangcheng on 2019/1/13.
//

#include "../system/Parameters.h"
#include "Eigen/Dense"

#ifndef LOCATION_SENSOR_H
#define LOCATION_SENSOR_H

using namespace Eigen;

class Sensor {
public:

    void Calibrate(MatrixXd &gyro_data,MatrixXd &acc_data,MatrixXd &mag_data,Parameters parameters);

};

#endif //LOCATION_SENSOR_H
