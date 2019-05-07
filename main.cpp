//#define CATCH_CONFIG_MAIN
//#include "catch.h"
//#include <iostream>
#include <Eigen/Dense>
#include "sensor/GPS.h"
#include <iomanip>
//#include "math/Quaternions.cpp"
#include "sensor/Gyroscope.cpp"
#include "models/AHRS.cpp"
#include "math/Optimizer.h"


#include <iostream>
#include <fstream>
#include <cassert>
#include <string>
#include "./test/utils/DataFormat.h"
#include "test/TestLocation.h"
#include "test/TestCalibration.h"
#include "config/Config.h"

using namespace Eigen;
using namespace std;

int main() {


    MatrixXd gyro2(8342,3),acc2(8342,3),mag2(8342,3), gps2(8342,7), g2(8342,3), ornt2(8342,3), road_data(8342,3);
    DataFormat dataFormat;
    dataFormat.readCSV(gyro2,acc2,mag2, gps2, g2, ornt2, road_data);
    TestLocation testLocation;
    testLocation.testLocation(gyro2, acc2, mag2, gps2, g2, ornt2, road_data);


//    Vector3d e(54.7,-0.3,   128.336);
//    Quaternions quaternions;
//    Vector4d q = quaternions.GetQFromEuler(e);
//    cout << q.transpose() << endl;
//    MatrixXd dcm = quaternions.GetDCMFromQ(q);
//    Vector3d gb(0.041, 8.009, 5.667);
//    Vector3d gn = dcm * gb;
//    cout << gn.transpose() << endl;
//
//    Location location;
//    Vector3d gyro_data_v(0.004263,0.019169,-0.001014);
//    Vector3d mag_data_v(-2.313675,-82.446960,-366.183838);
//    Vector3d acc_data_v(0.105081,0.108075,9.774973);
//    VectorXd gps_data_v(7);
//    gps_data_v << 114.174118,22.283789,0.0,0.0,24.0,0.0,1554348968704.665039;
//    Vector3d g_data_v(0.094139, 0.107857,9.808955);
//    Vector3d ornt_data_v(-0.549866,0.629957,-0.069398);
//    location.PredictCurrentPosition(gyro_data_v,acc_data_v,mag_data_v,gps_data_v,g_data_v,ornt_data_v);
//    cout <<     location.GetGNSSINS().lng << " " <<     location.GetGNSSINS().lat << endl;
    return 0;
}