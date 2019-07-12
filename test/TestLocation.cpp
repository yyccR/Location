//
// Created by yangcheng on 2019/1/17.
//

#include "TestLocation.h"
#include "TestCalibration.h"
#include "iostream"
#include "../sensor/Accelerometer.h"
#include "../sensor/GPS.h"
#include "../math/Quaternions.h"
#include "../test/utils/DataFormat.h"
using namespace Eigen;
using namespace routing;
//using namespace std;

void TestLocation::testLocation(MatrixXd &gyro_data, MatrixXd &acc_data, MatrixXd &mag_data, MatrixXd &gps_data, MatrixXd &g_data, MatrixXd &ornt_data, MatrixXd &road_data) {

//    // 初始状态
//    Quaternions quaternions;
//    Status status{};
//    status.Init();


    // 包括初始化状态
    Location location;
    location.SetHz(20.0);
//    DataFormat dataFormat;
//    dataFormat.writeCSV(acc_data, g_data, gyro_data, mag_data, ornt_data, gps_data);

    for(int i = 0; i < gyro_data.rows(); i++){
//    for(int i = 0; i < 13000; i++){
//
        Vector3d gyro_data_v = gyro_data.row(i);
        Vector3d mag_data_v = mag_data.row(i) / 1000.0;
        Vector3d acc_data_v = acc_data.row(i);
        VectorXd gps_data_v = gps_data.row(i);
        Vector3d g_data_v = g_data.row(i);
        Vector3d ornt_data_v = ornt_data.row(i);
        Vector3d road_data_v = road_data.row(i);


        location.PredictCurrentPosition(gyro_data_v, acc_data_v, mag_data_v, gps_data_v, g_data_v, ornt_data_v, road_data_v);
        GNSSINS gnssins = location.GetGNSSINS();
//
//
        std::cout.precision(7);
//        std::cout << location.status.position.lng << " " << location.status.position.lat << std::endl;
        std::cout << gnssins.lng << " " << gnssins.lat << " " << gnssins.bearing << std::endl;
    }

}
