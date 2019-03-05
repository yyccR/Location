//
// Created by yangcheng on 2019/1/13.
//

#include "Sensor.h"
#include "../sensor/Gyroscope.h"
#include "../sensor/Accelerometer.h"
#include "../sensor/Magnetometer.h"

using namespace Eigen;
using namespace routing;

/**
 * 传感器标定入口, 目前包含陀螺仪,地磁计,加速计
 *
 * @param gyro_data,陀螺仪数据, 采样建议: n>=200
 * @param acc_data, 加速计数据, 采样建议: 静止时采集6个轴转向180°各1组数据, 共6组数据
 * @param mag_data, 地磁计数据, 采样建议: 静止时采集6个轴转向180°各1组数据, 共6组数据
 * @param status
 */
void Sensor::Calibrate(MatrixXd &gyro_data, MatrixXd &acc_data, MatrixXd &mag_data, Status *status) {

    // 标定陀螺仪
    Gyroscope gyroscope;
    gyroscope.GyroCalibration(gyro_data, status);

    // 标定加速计
    Accelerometer accelerometer;
    accelerometer.AccCalibration(acc_data, status);

    // 标定地磁计
    Magnetometer magnetometer;
    magnetometer.MagCalibration(mag_data, status);
}