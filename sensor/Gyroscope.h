//
// Created by yangcheng on 2018/12/13.
//

#ifndef LOCATION_GYROSCOPE_H
#define LOCATION_GYROSCOPE_H


#include <eigen/Dense>

using namespace Eigen;

class Gyroscope {
public:

    // 从陀螺仪获取姿态旋转矩阵(方向余弦矩阵DCM), b系坐标转g系
    Matrix3d GetDCM(Vector3d &gyro, double &deltaT);

    // 姿态更新。
//    Vector3d UpdateAttitude(Matrix3d &dcm, Vector3d &state);
};


#endif //LOCATION_GYROSCOPE_H
