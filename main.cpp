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

using namespace Eigen;
using namespace std;

int main() {

//    MatrixXd gyro3(1731,3),acc3(1731,3),mag3(1731,3);
//    DataFormat dataFormat;
//    dataFormat.readCaliData(gyro3,acc3,mag3);
//    Status status;
//    TestCalibration testCalibration;
//    testCalibration.testCalibration(gyro3, acc3, mag3, &status);
//    cout << "acc coef " << status.parameters.acc_coef << endl;
//    cout << "gyro coef " << status.parameters.gyro_coef << endl;
//    cout << "mag coef " << status.parameters.mag_coef << endl;


    MatrixXd gyro2(3388,3),acc2(3388,3),mag2(3388,3), gps2(3388,6), g2(3388,3), ornt2(3388,3);
    VectorXd h(3388);
//    MatrixXd gyro2(3544,3),acc2(3544,3),mag2(3544,3), gps2(3544,6), g2(3544,3), ornt2(3544,3);
//    VectorXd h(3544);
    DataFormat dataFormat;
    dataFormat.readCSV(gyro2,acc2,mag2, gps2, g2, ornt2);
//    cout << gps2 << endl;
    TestLocation testLocation;
    testLocation.testLocation(gyro2, acc2, mag2, gps2, g2, ornt2);

//    Coordinate co;
//    Point2D p = co.LngLat2Mercator(-114.32894, 80.585748);
//    cout << setprecision(10) << p.lng << " | " << p.lat << endl;
//
//    Point2D p2 = co.Mercator2LngLat(12727039.383734727, 3579066.6894065146);
//    cout << setprecision(10) << p2.lng << "|" << p2.lat << endl;

//    Gyroscope gyroscope;

//    Vector3d v(0.1649, -0.1638, -0.0465);
//    double dt = 1.0;
//    Matrix3d dcm = gyroscope.GetDCM(v, dt);
//    cout << dcm << endl;
//
//    Vector3d s(-2.1069, 9.0692, 3.1028);
//    Vector3d ns = gyroscope.UpdateAttitude(dcm, s);
//    cout << ns << endl;
//
//    Vector3d g(0, 0, 9.80);
//    Vector3d gg = dcm.transpose() * g;
//    cout << gg << endl;

//    Vector3d init_w(63.1 / 180 * M_PI, 0.9 / 180.0 * M_PI, -129.83 / 180.0 * M_PI);
//    double t2 = 1.0;
//    Matrix3d init_dcm = gyroscope.GetDCM(init_w, t2);
////
//    Vector3d init_g(0,0,9.8);
//    Vector3d g2 = init_dcm.transpose() * init_g;
//    cout << g2 << endl;
//
//
//    Quaternions quaternions;
//    Vector4d eulerQ = quaternions.GetQFromEuler(init_w);
//    cout << "enler q = " << eulerQ << endl;
//    Matrix3d dcm_q = quaternions.GetDCMFromQ(eulerQ);
//    cout << "dcm q = " << dcm_q << endl;
//    Vector3d g3 = dcm_q.transpose() * init_g;
//    cout << "g3 = " << g3 << endl;



//    Vector3d a;
//    a(0) = 100.0;
//    a(1) = 100.0;
//    a(2) = 100.0;
////    a(0) = 10;
////    a(1) = 100;
////    a(2) =1000;
//    Vector3d *b = &a;
//    (*b)(0) = 200.0;
//    (*b)(1) = 300.0;
//    (*b)(2) = 400.0;
//    cout << "b ptr " << b << endl;
//    cout << "b ptra " << *b << endl;

//    cout << "Test AHRS algorithm." << endl;
//
//    Vector3d init_w(63.1 / 180 * M_PI, 0.9 / 180.0 * M_PI, -129.83 / 180.0 * M_PI);
//    Vector3d err(0.0,0.0,0.0);
//    Vector3d acc(-0.09,-0.01,9.81);
//    Vector3d mag(-0.0245,-0.0345,-0.0255);
//    Vector3d init_g(0,0,9.8);
//    double ki = 0.008;
//    double kp = 10.0;
//    double t = 0.001;
//    AHRS ahrs;
//    Quaternions quaternions;
//    Vector4d na = ahrs.UpdateAttitude(&err,init_w,acc,mag,ki,kp,t);
//    Vector4d eulerQ = quaternions.GetQFromEuler(init_w);
//    cout << " 初始姿态(欧拉角): \n" << init_w << endl;
//    cout << " 姿态角转四元数 \n" << eulerQ  << endl;
//    cout << " 利用加速计，地磁计较正姿态, 得到更新后的姿态四元数: \n" << na << endl;
//
//    Matrix3d newRotated_b2n = quaternions.GetDCMFromQ(na);
//    Vector3d newRotated_G = newRotated_b2n.transpose() * init_g;
//    cout << " 从较正后的姿态获取方向余弦矩阵（地理坐标系转机体坐标系） \n" << newRotated_b2n << endl;
//    cout << " 将改方向余弦用于旋转地心引力向量（0，0，9.8）上, 可得改力在机体上的分量为 \n" << newRotated_G << endl;
//
//
//
//    cout << "\n\n Test Optimizer, LM algorithm and GuassNewton algorithm. \n" << endl;
//    Optimizer optimizer;
//    MatrixXd data(6, 3);
//    data << 0.43,0.32,9.81,
//            0.21,0.25,9.79,
//            -0.13,0.12,9.82,
//            -0.01,-0.22,9.77,
//            0.44,-0.21,9.85,
//            0.51,-0.34,9.88;
//
//    data /= 9.8;
//
//    cout << "\n 标准化后的数据:" << endl;
//    cout << data << endl;
//
//    VectorXd coef(6);
//    VectorXd coef_nt(6);
//    coef << 0,0,0,1.0,1.0,1.0;
//    coef_nt << 0,0,0,1.0,1.0,1.0;
//
//    double gamma = 1.0;
//    double epsilon = 0.00001;
//    int max_step = 200;
//
//
//    cout << "\n 计算f(x):" << endl;
//    VectorXd fx = optimizer.EllipticalFx(data,&coef);
//    cout << fx << endl;
//
//    cout << "\n 计算jacobi:" << endl;
//    MatrixXd jacobi = optimizer.EllipticalCaliJacobi(data, &coef);
//    cout << jacobi << endl;
//
//    cout << "\n 初始coef: \n" << coef << endl;
//
//    optimizer.LevenbergMarquardt(data, &coef, gamma, epsilon, max_step);
//    cout << "\n LM优化后coef: \n" << coef << endl;
//
//    optimizer.GaussNewton(data, &coef_nt, epsilon, max_step);
//    cout << "\n GuassNewton优化后coef: \n" << coef_nt << endl;

    return 0;
}