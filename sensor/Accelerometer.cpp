//
// Created by yangcheng on 2018/11/28.
//

#include "Accelerometer.h"
#include "../math/Quaternions.h"

Vector3d Accelerometer::Normalise(Vector3d &a) const {
    Vector3d normA;
    double norm2 = a(0) * a(0) + a(1) * a(1) + a(2) * a(2);
    // 如果四元数各项足够接近单位四元数, 则不做任何处理.
    if (norm2 != 0.0) {
        double norm = sqrt(norm2);
        normA(0) = a(0) / norm;
        normA(1) = a(1) / norm;
        normA(2) = a(2) / norm;
    } else {
        normA = a;
    }
    return normA;
}

// 通过旋转获取 地理坐标系下重力加速度 转 机体坐标系下的重力加速度.
Vector3d Accelerometer::RotateG(Matrix3d &n2b) const {
    // b系下坐标
    Vector3d bg;
    // 地理坐标系下重力加速度
    Vector3d g(0, 0, 1);
    bg = n2b * g;
    return bg;
}

// 加速计向量(originA)叉乘地理重力转b系(rotatedG)误差，用于较正陀螺仪.
Vector3d Accelerometer::GetAccError(Vector3d &originA, Vector3d &rotatedG) const {
    Vector3d accErr;

    accErr(0) = originA(1) * rotatedG(2) - originA(2) * rotatedG(1);
    accErr(1) = originA(2) * rotatedG(0) - originA(0) * rotatedG(2);
    accErr(2) = originA(0) * rotatedG(1) - originA(1) * rotatedG(0);

    return accErr;
}

//Accelerometer::Accelerometer(double &x, double &y, double &z, Velocity &velocity) {
//    this->x = x;
//    this->y = y;
//    this->z = z;
//    this->velocity.vx = velocity.vx;
//    this->velocity.vy = velocity.vy;
//    this->velocity.vz = velocity.vz;
//}

Accelerometer::Accelerometer() {}

Accelerometer::~Accelerometer() = default;

//Accelerometer Accelerometer::Rotate(const Quaternions &quaternions, const Quaternions &quaternion_inv) const {
//    Quaternions QA(0, this->x, this->y, this->z);
//    Quaternions qleft = quaternions * QA;
//    Quaternions qright = qleft * quaternion_inv;
//    return Accelerometer(qright.x, qright.y, qright.z, this->velocity);
//}
//
//Point3D Accelerometer::PositionIntegral(Point3D &initPoint, double t) const {
//    Point3D outPosition;
//    double deltaX = this->velocity.vx * t + 0.5 * this->x * t * t;
//    double deltaY = this->velocity.vy * t + 0.5 * this->y * t * t;
//    double deltaZ = this->velocity.vz * t + 0.5 * this->z * t * t;
//
//    outPosition.lng = initPoint.lng + deltaX;
//    outPosition.lat = initPoint.lat + deltaY;
//    outPosition.altitude = initPoint.altitude + deltaZ;
//    return outPosition;
//}