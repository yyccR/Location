//
// Created by yangcheng on 2019/1/18.
//

#ifndef LOCATION_TESTCALIBRATION_H
#define LOCATION_TESTCALIBRATION_H

#include "Eigen/Dense"
#include "../system/Status.h"

using namespace Eigen;

class TestCalibration {
public:
    void testCalibration(MatrixXd &gyro_data, MatrixXd &acc_data, MatrixXd &mag_data, Status *status);
};


#endif //LOCATION_TESTCALIBRATION_H
