//
// Created by yangcheng on 2018/11/27.
//
#include "Eigen/Dense"

#ifndef LOCATION_QUATERNION_H
#define LOCATION_QUATERNION_H


class Quaternions {
public:

    Quaternions();

    virtual ~Quaternions();

    // 归一化.
    Eigen::Vector4d Normalise(Eigen::Vector4d &q) const;

    // 共轭四元数.
    Eigen::Vector4d GetConjugate(Eigen::Vector4d &q) const;

    // 四元数基本运算，加，点乘，叉乘
    // Quaternion operator*(const Quaternion &quaternion) const;
    Eigen::Vector4d Add(Eigen::Vector4d &q1, Eigen::Vector4d &q2) const;

    Eigen::Vector4d DotMulti(Eigen::Vector4d &q1, Eigen::Vector4d &q2) const;

    Eigen::Vector4d CrossMulti(Eigen::Vector4d &q1, Eigen::Vector4d &q2) const;

    // 从欧拉角v(x,y,z)中获取四元数Q(q0,q1,q2,q3).
    Eigen::Vector4d GetQFromEuler(Eigen::Vector3d &euler_angle) const;

    // 从四元数获取余弦矩阵DCM
    Eigen::Matrix3d GetDCMFromQ(Eigen::Vector4d &q);

    // 从四元数获取欧拉角
    Eigen::Vector3d GetEulerFromQ(Eigen::Vector4d &q);

};


#endif //LOCATION_QUATERNION_H
