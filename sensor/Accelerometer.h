//
// Created by yangcheng on 2018/11/28.
//
#include "../math/Quaternions.h"
#include "../math/Coordinate.h"


#ifndef LOCATION_ACCELEROMETER_H
#define LOCATION_ACCELEROMETER_H


//struct Velocity {
//    double vx;
//    double vy;
//    double vz;
//};


class Accelerometer {
public:

    // accelerate data
//    double x, y, z;
//    Velocity velocity;

//    Accelerometer(double &x, double &y, double &z, Velocity &velocity);
    Accelerometer();

    virtual ~Accelerometer();

    // 归一化.
    Vector3d Normalise(Vector3d &a) const;

    // 通过旋转获取 地理坐标系下重力加速度 转 机体坐标系下的重力加速度.
    Vector3d RotateG(Matrix3d &n2b) const;

    // 加速计向量(originA)叉乘地理重力转b系(rotatedG)误差，用于较正陀螺仪
    Vector3d GetAccError(Vector3d &originA, Vector3d &rotatedG) const;

    // rotate the accelerate data into Geo coordinates.
//    Accelerometer Rotate(const Quaternions &quaternions, const Quaternions &quaternion_inv) const;

//    // format the Accelerate data, correct the error and unit.
//    void Correct();
//
//    // delete the affection from gravity.
//    void DeleteGravity();

    // position integral.
//    Point3D PositionIntegral(Point3D &initPoint, double t) const;
};


#endif //LOCATION_ACCELEROMETER_H
