//
// Created by yangcheng on 2019/1/14.
//

#include "../sensor/Accelerometer.h"
#include "../models/AHRS.h"
#include "Location.h"

/**
 * 定位,计算当前位置
 *
 * @param gyro_data, 陀螺仪原始数据, w(x,y,z)
 * @param acc_data, 加速计原始数据, a(x,y,z)
 * @param mag_data, 地磁计原始数据, m(x,y,z)
 * @param status, 状态容器, 包含位置,姿态,速度,参数等信息
 * @param t, 采样时间,(1/采样频率)
 */
void Location::PredictCurrentPosition(Vector3d &gyro_data, Vector3d &acc_data, Vector3d &mag_data, Status *status,
                                      double t) {

    // 传感器参数
    Parameters parameters = (*status).parameters;
    // 利用标定参数较正当前传感器数据
    Vector3d gyro_data_cali;
    Vector3d acc_data_cali;
    Vector3d mag_data_cali;
    // 较正陀螺仪
    gyro_data_cali(0) = gyro_data(0) - parameters.gyro_coef(0);
    gyro_data_cali(1) = gyro_data(1) - parameters.gyro_coef(1);
    gyro_data_cali(2) = gyro_data(2) - parameters.gyro_coef(2);
    // 较正加速计, 单位为g
    Vector3d acc_data_format = acc_data / (*status).parameters.g;
    acc_data_cali(0) = (acc_data_format(0) - parameters.acc_coef(0)) / parameters.acc_coef(3);
    acc_data_cali(1) = (acc_data_format(1) - parameters.acc_coef(1)) / parameters.acc_coef(4);
    acc_data_cali(2) = (acc_data_format(2) - parameters.acc_coef(2)) / parameters.acc_coef(5);
    // 较正地磁计
    mag_data_cali(0) = (mag_data(0) - parameters.mag_coef(0)) / parameters.mag_coef(3);
    mag_data_cali(1) = (mag_data(1) - parameters.mag_coef(1)) / parameters.mag_coef(4);
    mag_data_cali(2) = (mag_data(2) - parameters.mag_coef(2)) / parameters.mag_coef(5);

    // 欧拉角累计
    (*status).attitude.roll += mag_data_cali(0) * t;
    (*status).attitude.pitch += mag_data_cali(1) * t;
    (*status).attitude.yaw += mag_data_cali(2) * t;
    // 欧拉角
    Vector3d euler_angle;
    euler_angle(0) = (*status).attitude.roll;
    euler_angle(1) = (*status).attitude.pitch;
    euler_angle(2) = (*status).attitude.yaw;

    // 获取姿态
    AHRS ahrs;
    Vector3d *err = &(*status).parameters.err;
    double ki = (*status).parameters.ki;
    double kp = (*status).parameters.kp;
    double halfT = (*status).parameters.halfT;
    Vector4d attitude = ahrs.UpdateAttitude(err, euler_angle, acc_data_cali, mag_data_cali, ki, kp, halfT);

    // 加速计从b系转到n系
    Quaternions quaternions;
    Matrix3d newRotated_b2n = quaternions.GetDCMFromQ(attitude);
    Vector3d acc_n = newRotated_b2n * acc_data_cali;
    // 减去地心重力影响
    Vector3d v(0,0,1);
    Vector3d final_acc = (acc_n - v) * (*status).parameters.g;

    // 更新位置
    Accelerometer accelerometer;
    accelerometer.PositionIntegral(status, final_acc, t);

}