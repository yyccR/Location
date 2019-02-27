//
// Created by yangcheng on 2019/1/14.
//

#include "../sensor/GPS.h"
#include "../sensor/Accelerometer.h"
#include "../models/AHRS.h"
#include "Location.h"

using namespace Eigen;

/**
 * Location 初始化。
 */
Location::Location() {
    this->status.Init();
}

/**
 * 定位,计算当前位置
 *
 * @param gyro_data, 陀螺仪原始数据, w(x,y,z)
 * @param acc_data, 加速计原始数据, a(x,y,z)
 * @param mag_data, 地磁计原始数据, m(x,y,z)
 * @param gps_data, GPS原始数据, gps(lng,lat,alt,accuracy,speed,bearing)
 * @param g_data, 重力感应数据, g(x,y,z)
 * @param ornt_data, 方向传感器数据, o(roll,pitch,yaw)
 * @param status, 状态容器, 包含位置,姿态,速度,参数等信息
 */
void Location::PredictCurrentPosition(Vector3d &gyro_data, Vector3d &acc_data, Vector3d &mag_data, VectorXd &gps_data,
                                      Vector3d &g_data, Vector3d &ornt_data, Status *status) {

    // 传感器参数
    Parameters parameters = (*status).parameters;
    // 利用标定参数较正当前传感器数据
    Vector3d gyro_data_cali;
    Vector3d acc_data_cali;
    Vector3d mag_data_cali;
    Vector3d g_data_format;
    // 较正陀螺仪, 单位rad
    gyro_data_cali(0) = gyro_data(0) - parameters.gyro_coef(0);
    gyro_data_cali(1) = gyro_data(1) - parameters.gyro_coef(1);
    gyro_data_cali(2) = gyro_data(2) - parameters.gyro_coef(2);
    // 较正加速计, 单位为g
    acc_data_cali(0) = (acc_data(0) / (*status).parameters.g - parameters.acc_coef(0)) * parameters.acc_coef(3);
    acc_data_cali(1) = (acc_data(1) / (*status).parameters.g - parameters.acc_coef(1)) * parameters.acc_coef(4);
    acc_data_cali(2) = (acc_data(2) / (*status).parameters.g - parameters.acc_coef(2)) * parameters.acc_coef(5);
    // 重力数据归一
    g_data_format = g_data / (*status).parameters.g;
    // 较正地磁计, 单位μt
    mag_data_cali(0) = (mag_data(0) / 1000.0 - parameters.mag_coef(0)) * parameters.mag_coef(3);
    mag_data_cali(1) = (mag_data(1) / 1000.0 - parameters.mag_coef(1)) * parameters.mag_coef(4);
    mag_data_cali(2) = (mag_data(2) / 1000.0 - parameters.mag_coef(2)) * parameters.mag_coef(5);

    Vector3d g(0, 0, 1.0);

    // 获取姿态
    AHRS ahrs;
    Vector4d q_attitude = (*status).attitude.q_attitude;
    Vector3d *err = &(*status).parameters.err;
    double ki = (*status).parameters.ki;
    double kp = (*status).parameters.kp;
    double halfT = (*status).parameters.halfT;
    Vector4d attitude = ahrs.UpdateAttitude(err, q_attitude, gyro_data_cali, g_data, mag_data_cali, ki, kp, halfT);
    // 更新姿态
    (*status).attitude.q_attitude = attitude;

    // 加速计从b系转到n系
    Quaternions quaternions;
    Matrix3d newRotated_b2n = quaternions.GetDCMFromQ(attitude);
    Vector3d acc_b = acc_data_cali - g_data_format;
    Vector3d final_acc = newRotated_b2n * acc_b * (*status).parameters.g;

    // 记录起始位置和当前位置
    double start_x = (*status).position.x;
    double start_y = (*status).position.y;
//    double start_z = (*status).position.z;
    double start_lng = (*status).position.lng;
    double start_lat = (*status).position.lat;

    // 更新惯性位置,速度
    Accelerometer accelerometer;
    accelerometer.PositionIntegral(status, final_acc, (*status).parameters.t);

    // 获取GPS精度
    GPS gps;
    double gps_accuracy = gps_data(3);
    // 计算传感器运动距离
    double end_x = (*status).position.x;
    double end_y = (*status).position.y;
    double distance = sqrt((end_x - start_x) * (end_x - start_x) + (end_y - start_y) * (end_y - start_y));
    // 计算GPS运动距离
    double end_Lng = gps_data(0);
    double end_Lat = gps_data(1);
    double gps_move_dist = gps.CalDistance(start_lng, start_lat, end_Lng, end_Lat);
    // 判断是否相同时间间隔GPS移动与惯导相差不大,用于判断该点是否被采用
    bool is_gps_move_not_accpetted =  ceil((*status).parameters.ins_count / 10.0) * gps_move_dist <
            (*status).parameters.move_distance_threshod * (*status).parameters.ins_count;

    // 判断是否采用GPS数据
    if (gps_accuracy > (*status).parameters.weak_gps || (gps_data(0) == 0.0 && gps_data(1) == 0.0) || is_gps_move_not_accpetted) {
        // 采用惯导更新经纬度
        // 获取航向角
        double heading = ornt_data(2);
        // 计算航向角
        Vector2d gps_new = gps.CalDestination(start_lng, start_lat, distance, heading);
        // 更新经纬度
        (*status).position.lng = gps_new(0);
        (*status).position.lat = gps_new(1);
    } else {
        // 采用GPS数据更新经纬度
        (*status).position.lng = gps_data(0);
        (*status).position.lat = gps_data(1);
        (*status).position.altitude = gps_data(2);
        double gps_speed = gps_data(4);
        double gps_bearing = gps_data(5);
        gps.UpdateVelocity(status, gps_speed, gps_bearing);
        // 每个x,y,z都是相对与上一个准确的GPS数据。
        (*status).position.x = 0.0;
        (*status).position.y = 0.0;
        (*status).position.z = 0.0;
    }

}

/**
 * 获取当前位置
 * @return
 */
Position Location::GetCurrentPosition() {
    return  this->status.position;
}