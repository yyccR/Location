//
// Created by yangcheng on 2019/1/17.
//

#include "TestLocation.h"
#include "../system/Status.h"
#include "iostream"

using namespace Eigen;
using namespace std;

void TestLocation::testLocation(MatrixXd &gyro_data, MatrixXd &acc_data, MatrixXd &mag_data) {

//    cout << gyro_data << endl;
    // 初始状态
    Status status{};
    status.position.x = 0.0;
    status.position.y = 0.0;
    status.position.z = 0.0;

    status.attitude.roll = 0.78;
    status.attitude.pitch = 0.68;
    status.attitude.yaw = 6.86;

    status.velocity.v_x = 0.0;
    status.velocity.v_y = 0.0;
    status.velocity.v_z = 0.0;

    Vector3d gyro_coef(0.0,0.0,0.0);
    status.parameters.gyro_coef = gyro_coef;
    VectorXd acc_coef(6);
    acc_coef << 0.0,0.0,0.0,1.0,1.0,1.0;
    status.parameters.acc_coef = acc_coef;
    VectorXd mag_coef(6);
    mag_coef << 0.0,0.0,0.0,1.0,1.0,1.0;
    status.parameters.mag_coef = acc_coef;

    status.parameters.gamma = 1.0;
    status.parameters.epsilon = 0.000001;
    status.parameters.max_step = 200;
    Vector3d err(0.0,0.0,0.0);
    status.parameters.err = err;
    status.parameters.ki = 0.008;
    status.parameters.kp = 10.0;
    status.parameters.halfT = 1 / 40.0;
    status.parameters.g = 9.7847;
    double t = 1 / 20.0;

    Location location;
//    cout << status.parameters.err << endl;
    for(int i = 0; i < gyro_data.rows(); i++){

        Vector3d gyro_data_v = gyro_data.row(i);
        Vector3d mag_data_v = mag_data.row(i);
        Vector3d acc_data_v = acc_data.row(i);
//        cout << gyro_data_v << endl;
//        cout << mag_data_v << endl;
//        cout << acc_data_v << endl;

        location.PredictCurrentPosition(gyro_data_v, acc_data_v, mag_data_v, &status, t);
        cout << status.position.x << " " << status.position.y << " " << status.position.z << endl;
    }

}
