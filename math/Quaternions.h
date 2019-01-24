//
// Created by yangcheng on 2018/11/27.
//
#include "Eigen/Dense"

#ifndef LOCATION_QUATERNION_H
#define LOCATION_QUATERNION_H

using namespace Eigen;

class Quaternions {
public:

    Quaternions();

    virtual ~Quaternions();

    // 归一化.
    Vector4d Normalise(Vector4d &q) const;

    // 共轭四元数.
    Vector4d GetConjugate(Vector4d &q) const;

    // 四元数基本运算，加，点乘，叉乘
    // Quaternion operator*(const Quaternion &quaternion) const;
    Vector4d Add(Vector4d &q1, Vector4d &q2) const;

    Vector4d DotMulti(Vector4d &q1, Vector4d &q2) const;

    Vector4d CrossMulti(Vector4d &q1, Vector4d &q2) const;

    // 从欧拉角v(x,y,z)中获取四元数Q(q0,q1,q2,q3).
    Vector4d GetQFromEuler(Vector3d &euler_angle) const;

    // 从四元数获取余弦矩阵DCM
    Matrix3d GetDCMFromQ(Vector4d &q);

    // 从四元数获取欧拉角
    Vector3d GetEulerFromQ(Vector4d &q);

};


#endif //LOCATION_QUATERNION_H
