//
// Created by yangcheng on 2018/11/28.
//
#include "../math/Quaternions.h"
#include "../system/Status.h"


#ifndef LOCATION_ACCELEROMETER_H
#define LOCATION_ACCELEROMETER_H


class Accelerometer {
public:

    // accelerate data
//    double x, y, z;
//    Velocity velocity;

//    Accelerometer(double &x, double &y, double &z, Velocity &velocity);
    Accelerometer();

    virtual ~Accelerometer();

    // 归一化.
    Eigen::Vector3d Normalise(Eigen::Vector3d &a) const;

    // 通过旋转获取 地理坐标系下重力加速度 转 机体坐标系下的重力加速度.
    Eigen::Vector3d RotateG(Eigen::Matrix3d &n2b) const;

    // 加速计向量(originA)叉乘地理重力转b系(rotatedG)误差，用于较正陀螺仪
    Eigen::Vector3d GetAccError(Eigen::Vector3d &originA, Eigen::Vector3d &rotatedG) const;

    Eigen::Vector3d GetAccError(Eigen::Vector3d &originA, Eigen::Vector4d &q) const;

    // 加速计标定
    void AccCalibration(Eigen::MatrixXd &input_data, routing::Status *status);

    // rotate the accelerate data into Geo coordinates.
//    Accelerometer Rotate(const Quaternions &quaternions, const Quaternions &quaternion_inv) const;

//    // format the Accelerate data, correct the error and unit.
//    void Correct();
//
//    // delete the affection from gravity.
//    void DeleteGravity();

    // position integral.
    void PositionIntegral(routing::Status *status, Eigen::Vector3d &acc, double t);

    void StrapdownUpdateVelocityPosition(routing::Status *status, Eigen::Vector3d &acc,
                                         Eigen::Vector4d &q_attitude, Eigen::Vector3d &g_data);

    // 用于加速计过滤数据
    Eigen::Vector3d FilterData(routing::Status *status, Eigen::Vector3d &acc_data);
};


#endif //LOCATION_ACCELEROMETER_H
