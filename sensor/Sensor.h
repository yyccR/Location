//
// Created by yangcheng on 2019/1/13.
//

#include "../system/Status.h"
#include "Eigen/Dense"

#ifndef LOCATION_SENSOR_H
#define LOCATION_SENSOR_H


class Sensor {
public:

    void Calibrate(Eigen::MatrixXd &gyro_data,Eigen::MatrixXd &acc_data,Eigen::MatrixXd &mag_data, routing::Status *status);

};

#endif //LOCATION_SENSOR_H
