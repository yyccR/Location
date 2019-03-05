//
// Created by yangcheng on 2019/1/18.
//

#include "TestCalibration.h"
#include "../sensor/Sensor.h"
#include "iostream"

using namespace Eigen;
using namespace routing;

void TestCalibration::testCalibration(MatrixXd &gyro_data, MatrixXd &acc_data, MatrixXd &mag_data, Status *status) {

    Vector3d gyro_coef(0.0,0.0,0.0);
    (*status).parameters.gyro_coef = gyro_coef;
    VectorXd acc_coef(6);
    acc_coef << 0.0,0.0,0.0,1.0,1.0,1.0;
    (*status).parameters.acc_coef = acc_coef;
    VectorXd mag_coef(6);
    mag_coef << 0.0,0.0,0.0,1.0,1.0,1.0;
    (*status).parameters.mag_coef = mag_coef;

    (*status).parameters.gamma = 1.0;
    (*status).parameters.epsilon = 0.0000000001;
    (*status).parameters.max_step = 500;
    Vector3d err(0.0,0.0,0.0);
    (*status).parameters.err = err;
    (*status).parameters.ki = 0.0001;
    (*status).parameters.kp = 300.0;
    (*status).parameters.halfT = 1 / 20.0;
    (*status).parameters.g = 9.805567;
    (*status).parameters.mag = 0.15744;

    Sensor sensor;

    // acc数据调整
//    MatrixXd acc_data_adj(12, 3);
    // mag数据调整
//    MatrixXd mag_data_adj(12, 3);

//    acc_data = acc_data / (*status).parameters.g;

//    for (int i = 0; i < 12; i++) {
//        // 加速计数据单位为g
//        acc_data_adj.block(i, 0, 1, 3) = acc_data.block(i * 5, 0, 1, 3) / (*status).parameters.g;
//        mag_data_adj.block(i, 0, 1, 3) = mag_data.block(i * 5, 0, 1, 3);
//    }

//    cout << gyro_data << endl;
//    cout << "------------------" << endl;
//    cout << acc_data << endl;
//    cout << "--------------" << endl;
//    cout << mag_data << endl;
    sensor.Calibrate(gyro_data, acc_data, mag_data, status);

}