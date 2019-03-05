//
// Created by yangcheng on 2019/1/18.
//

#ifndef LOCATION_TESTCALIBRATION_H
#define LOCATION_TESTCALIBRATION_H

#include "Eigen/Dense"
#include "../system/Status.h"


class TestCalibration {
public:
    void testCalibration(Eigen::MatrixXd &gyro_data, Eigen::MatrixXd &acc_data, Eigen::MatrixXd &mag_data, routing::Status *status);
};


#endif //LOCATION_TESTCALIBRATION_H
