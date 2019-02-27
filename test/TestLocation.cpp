//
// Created by yangcheng on 2019/1/17.
//

#include "TestLocation.h"
#include "TestCalibration.h"
#include "iostream"
#include "../sensor/Accelerometer.h"
#include "../sensor/GPS.h"
#include "../math/Quaternions.h"
using namespace Eigen;
//using namespace std;

void TestLocation::testLocation(MatrixXd &gyro_data, MatrixXd &acc_data, MatrixXd &mag_data, MatrixXd &gps_data, MatrixXd &g_data, MatrixXd &ornt_data) {

    // 初始状态
    Quaternions quaternions;
    Status status{};
    status.Init();


    Location location;
    for(int i = 0; i < gyro_data.rows(); i++){

        Vector3d gyro_data_v = gyro_data.row(i);
        Vector3d mag_data_v = mag_data.row(i);
        Vector3d acc_data_v = acc_data.row(i);
        VectorXd gps_data_v = gps_data.row(i);
        Vector3d g_data_v = g_data.row(i);
        Vector3d ornt_data_v = ornt_data.row(i);


        location.PredictCurrentPosition(gyro_data_v, acc_data_v, mag_data_v, gps_data_v, g_data_v, ornt_data_v,&status);
        Position position  = status.GetPosition();

        std::cout.precision(9);
        std::cout << status.position.lng << " " << status.position.lat << std::endl;
    }

}
