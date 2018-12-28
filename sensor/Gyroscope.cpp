//
// Created by yangcheng on 2018/12/13.
//

#include "Gyroscope.h"
#include <Eigen/Dense>
#include <math.h>

using namespace Eigen;

// 从陀螺仪获取姿态旋转矩阵(方向余弦矩阵DCM), b系坐标转g系
Matrix3d Gyroscope::GetDCM(Vector3d &gyro, double &deltaT) {

    // 计算deltaT时间内角度变化值，deltaT越小越精确。
    double delta_wx = gyro(0) * deltaT;
    double delta_wy = gyro(1) * deltaT;
    double delta_wz = gyro(2) * deltaT;

    double cos_delta_wx = cos(delta_wx);
    double sin_delta_wx = sin(delta_wx);
    double cos_delta_wy = cos(delta_wy);
    double sin_delta_wy = sin(delta_wy);
    double cos_delta_wz = cos(delta_wz);
    double sin_delta_wz = sin(delta_wz);

    // 计算DCM矩阵(z * y * x)。
    Matrix3d DCM;
    DCM(0, 0) = cos_delta_wy * cos_delta_wz;
    DCM(0, 1) = -sin_delta_wz * cos_delta_wx + sin_delta_wx * sin_delta_wy * cos_delta_wz;
    DCM(0, 2) = sin_delta_wx * sin_delta_wz + sin_delta_wy * cos_delta_wx * cos_delta_wz;
    DCM(1, 0) = sin_delta_wz * cos_delta_wy;
    DCM(1, 1) = cos_delta_wx * cos_delta_wz + sin_delta_wx * sin_delta_wy * sin_delta_wz;
    DCM(1, 2) = -sin_delta_wx * cos_delta_wz + sin_delta_wy * sin_delta_wz * cos_delta_wx;
    DCM(2, 0) = -sin_delta_wy;
    DCM(2, 1) = sin_delta_wx * cos_delta_wy;
    DCM(2, 2) = cos_delta_wx * cos_delta_wy;

    return DCM;
}
//
//// 姿态更新。
//Vector3d Gyroscope::UpdateAttitude(Matrix3d &dcm, Vector3d &state) {
//
//    Vector3d newState;
//    // 旋转更新.
//    newState = dcm * state;
//    return newState;
//}