#include <Eigen/Dense>
#include "sensor/GPS.h"
#include <iomanip>
#include "math/Optimizer.h"
#include <iostream>
#include <fstream>
#include <cassert>
#include <string>
#include "test/utils/DataFormat.h"
#include "test/TestLocation.h"
#include "test/TestCalibration.h"
#include "config/Config.h"
#include "Location.h"
#include "XgboostDetector.h"
#include "math/Quaternions.h"
#include <chrono>
#include "TestXgboostDetector.h"

using namespace Eigen;
using namespace std;

int main() {

//    TestXgboostDetector testXgboostDetector;
//    testXgboostDetector.TestDetector();

//    using namespace std::chrono;
//    long long int ms = duration_cast< milliseconds >(
//            system_clock::now().time_since_epoch()
//    ).count();
//    // testing xgboost model.
//    std::string model_path = "D:\\worksheet\\clion\\Location\\models\\xgboost_model.txt";
//    StopDetection stopDetection = XgboostDetector(model_path);
//    Eigen::VectorXd input(27);
//    input << 0.166704003,0.793647502,0.585092658,-0.836314314,-0.495913423,0.233769642,0.202316318,0.528412458,0.824529188,-0.85625963,-0.222778842,0.466035443,0.221061031,-0.715468667,-0.662749279,0.922008461,0.260256615,0.28664768,3.344844582,0.94415395,1.039894947,2.136208297,-1.528755739,-2.820955601,-0.750271,2.003565,-1.070326;
//    bool res = stopDetection.IsStopping(input);
//    long long int ms2 = duration_cast< milliseconds >(
//            system_clock::now().time_since_epoch()
//    ).count();
//    std::cout << "xgboost detector, current status is stopping: " << res
//              << " using time(ms) " << ms2 - ms << std::endl;

    // unit test
//    int data_size = 45461;
//    MatrixXd gyro2(data_size,3),acc2(data_size,3),mag2(data_size,3), gps2(data_size,7),
//            g2(data_size,3), ornt2(data_size,3), road_data(data_size,3);
//    DataFormat dataFormat;
//    dataFormat.writeCSVs();
//    std::string file = "D:\\worksheet\\clion\\Location\\test\\data\\sensor_log\\origin_sensors_data_1558592729003.9297.csv";
//    dataFormat.readCSV(file, gyro2,acc2,mag2, gps2, g2, ornt2, road_data);
//    TestLocation testLocation;
//    testLocation.testLocation(gyro2, acc2, mag2, gps2, g2, ornt2, road_data);

     checking sensor.
    Vector3d e(57.221,-0.543, 143.2);
    Quaternions quaternions;
    Vector4d q = quaternions.GetQFromEuler(e);
//    cout << q.transpose() << endl;
    MatrixXd dcm = quaternions.GetDCMFromQ(q);
    Vector3d gb(0.041, 8.248, 5.311);
    Vector3d gn = dcm * gb;
    cout << "Using euler rotate the gravity from b frame: [" << gb.transpose()
         << "] to n frame, result is: [" << gn.transpose() << "]." << endl;


     testing work flow.
    Location location;
    Vector3d gyro_data_v(0.004263,0.019169,-0.001014);
    Vector3d mag_data_v(-2.313675,-82.446960,-366.183838);
    Vector3d acc_data_v(0.105081,0.108075,9.774973);
    VectorXd gps_data_v(7);
    gps_data_v << 114.174118,22.283789,0.0,0.0,24.0,0.0,1554348968704.665039;
    Vector3d g_data_v(0.094139, 0.107857,9.808955);
    Vector3d ornt_data_v(-0.549866,0.629957,-0.069398);
    Vector3d road_data(1000.0, 0.0, 0);
    location.PredictCurrentPosition(gyro_data_v,acc_data_v,mag_data_v,gps_data_v,g_data_v,ornt_data_v, road_data);
    cout << "Current predict result: lng " << location.GetGNSSINS().lng << ", lat " <<     location.GetGNSSINS().lat << endl;
    return 0;
}