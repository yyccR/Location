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

void TestLocation::testLocation(MatrixXd &gyro_data, MatrixXd &acc_data, MatrixXd &mag_data, MatrixXd &gps_data, MatrixXd &g_data, VectorXd &heading) {

    // 初始状态
    Quaternions quaternions;
    Status status{};
    status.position.x = 0.0;
    status.position.y = 0.0;
    status.position.z = 0.0;

//    status.attitude.roll = 3.14;
    status.attitude.roll = 0.68;
//    status.attitude.pitch = -6.66;
    status.attitude.pitch = 0.78;
//    status.attitude.yaw = -6.255260039;
    status.attitude.yaw = 0.119729587;
    Vector3d init_euler_angle(status.attitude.roll, status.attitude.pitch, status.attitude.yaw);
    status.attitude.q_attitude = quaternions.GetQFromEuler(init_euler_angle);

    status.velocity.v_x = 0.0;
    status.velocity.v_y = 0.0;
    status.velocity.v_z = 0.0;

    Vector3d gyro_coef(0.000943732,0.00123328,-0.000637955);
    status.parameters.gyro_coef = gyro_coef;
    VectorXd acc_coef(6);
    acc_coef << 0.0,0.0,0.0,1.0,1.0,1.0;
//    acc_coef << -0.00942071,-0.0048024,-0.0100331,1.00605,1.00022,1.00478;
    status.parameters.acc_coef = acc_coef;
    VectorXd mag_coef(6);
    mag_coef << 0.0,0.0,0.0,1.0,1.0,1.0;
//    mag_coef << 0.000118878,-8.06472e-05,-0.000500984,0.992046,0.996096,0.841067;
//    mag_coef << 0.0126454,-0.0598358,-0.128009,1.01087,1.36305,0.816457;
    status.parameters.mag_coef = mag_coef;

    status.parameters.weak_gps = 100;
    status.parameters.gamma = 1.0;
    status.parameters.epsilon = 0.000001;
    status.parameters.max_step = 200;
    Vector3d err(0.0,0.0,0.0);
    status.parameters.err = err;
    status.parameters.ki = 0.05;
    status.parameters.kp = 10.0;
    status.parameters.halfT = 1 / 20.0;
    status.parameters.g = 9.805567;
    status.parameters.mag = 157.44;
    double t = 1 / 3.5;

//    TestCalibration testCalibration;
//    testCalibration.testCalibration(gyro_data, acc_data, mag_data, &status);
//    cout << "acc coef " << status.parameters.acc_coef << endl;
//    cout << "gyro coef " << status.parameters.gyro_coef << endl;
//    cout << "mag coef " << status.parameters.mag_coef << endl;

    Location location;
    for(int i = 0; i < gyro_data.rows(); i++){

        Vector3d gyro_data_v = gyro_data.row(i);
        Vector3d mag_data_v = mag_data.row(i);
        Vector3d acc_data_v = acc_data.row(i);
        Vector4d gps_data_v = gps_data.row(i);
        Vector3d g_data_v = g_data.row(i);
        double h = heading(i);

//        if(abs(gyro_data(0)) >=  0.04 || abs(gyro_data(1)) >=  0.04 ||  abs(gyro_data(2)) >=  0.04) {
//            status.attitude.roll += gyro_data(0) * t;
//            status.attitude.pitch += gyro_data(1) * t;
//            status.attitude.yaw += gyro_data(2) * t;
//        }
//
//        Vector3d euler_angle;
//        euler_angle(0) = status.attitude.pitch;
//        euler_angle(1) = status.attitude.roll;
//        euler_angle(2) = status.attitude.yaw;
//
//        // 从欧拉角获取四元数
//        Quaternions quaternions;
//        Vector4d euler_q = quaternions.GetQFromEuler(euler_angle);
//        Vector4d attitude = quaternions.GetQFromEuler(euler_angle);
//        // 计算旋转矩阵(b系到n系)
//        Matrix3d b2n = quaternions.GetDCMFromQ(euler_q);
//        Matrix3d n2b = b2n.transpose();
//
//        Vector3d rotate_g = n2b * g_data_v;
//        Vector3d real_acc = acc_data_v - rotate_g;
//        Vector3d n_acc = b2n * real_acc;
//
//        // 记录起始位置和当前位置
//        double start_x = status.position.x;
//        double start_y = status.position.y;
//        double start_lng = status.position.lng;
//        double start_lat = status.position.lat;
//
//        // 更新惯性位置,速度
//        Accelerometer accelerometer;
//        accelerometer.PositionIntegral(&status, n_acc, t);
//        // 更新地理坐标位置
//        GPS gps;
//        double gps_accuracy = gps_data_v(3);
//        if (gps_accuracy > status.parameters.weak_gps) {
//            // 计算距离
//            double end_x = status.position.x;
//            double end_y = status.position.y;
//            double distance = sqrt((end_x - start_x) * (end_x - start_x) + (end_y - start_y) * (end_y - start_y));
//            // 计算航向角
//            Vector3d euler = quaternions.GetEulerFromQ(attitude);
//            double heading = euler(2);
//            // 计算航向角
//            Vector2d gps_new = gps.CalDestination(start_lng, start_lat, distance, heading);
////        std::cout.precision(9);
////        std::cout << " acc cali " << acc_data_cali.transpose()
////                << " acc n " << acc_n.transpose()
////                << " final acc " << final_acc.transpose()
//////                  << " distance " << distance << " heading " << heading / M_PI * 180.0 << " gps_new " << gps_new.transpose()
////                  << " v = " << (*status).velocity.v_x << " " << (*status).velocity.v_y << " " << (*status).velocity.v_z
////                  << std::endl;
//            // 更新经纬度
//            status.position.lng = gps_new(0);
//            status.position.lat = gps_new(1);
//        } else {
//            status.position.lng = gps_data_v(0);
//            status.position.lat = gps_data_v(1);
//            status.position.altitude = gps_data_v(2);
//        }



        location.PredictCurrentPosition(gyro_data_v, acc_data_v, mag_data_v, gps_data_v, g_data_v, h,&status, t);
        std::cout.precision(9);
        std::cout << status.position.lng << " " << status.position.lat << std::endl;
    }

}
