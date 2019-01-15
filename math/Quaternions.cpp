//
// Created by yangcheng on 2018/11/27.
//

#include <cmath>
#include "Eigen/Dense"
#include "Quaternions.h"


Quaternions::Quaternions() {};

Quaternions::~Quaternions() = default;

// 归一化.
Vector4d Quaternions::Normalise(Vector4d &q) const {
    Vector4d normQ;
    double norm2 = q(0) * q(0) + q(1) * q(1) + q(2) * q(2) + q(3) * q(3);
    // 如果四元数各项足够接近单位四元数, 则不做任何处理.
    if (norm2 != 0.0) {
        double norm = sqrt(norm2);
        normQ(0) = q(0) / norm;
        normQ(1) = q(1) / norm;
        normQ(2) = q(2) / norm;
        normQ(3) = q(3) / norm;
    } else {
        normQ = q;
    }
    return normQ;
}

// 共轭四元数, 实部相同，虚部取反.
Vector4d Quaternions::GetConjugate(Vector4d &q) const {
    Vector4d conjQ;
    conjQ(0) = q(0);
    conjQ(1) = -q(1);
    conjQ(2) = -q(2);
    conjQ(3) = -q(3);
    return conjQ;
}

// 四元数基本运算, 加.
Vector4d Quaternions::Add(Vector4d &q1, Vector4d &q2) const {
    Vector4d addRes;

    addRes(0) = q1(0) + q2(0);
    addRes(1) = q1(1) + q2(1);
    addRes(2) = q1(2) + q2(2);
    addRes(3) = q1(3) + q2(3);

    return addRes;
}

// 四元数基本运算, 点乘.
Vector4d Quaternions::DotMulti(Vector4d &q1, Vector4d &q2) const {
    Vector4d dotMultiRes;

    dotMultiRes(0) = q1(0) * q2(0);
    dotMultiRes(1) = q1(1) * q2(1);
    dotMultiRes(2) = q1(2) * q2(2);
    dotMultiRes(3) = q1(3) * q2(3);

    return dotMultiRes;
}

// 四元数基本运算, 叉乘.
Vector4d Quaternions::CrossMulti(Vector4d &q1, Vector4d &q2) const {
    Vector4d crossMultiRes;

    crossMultiRes(0) = q1(0) * q2(0) - q1(1) * q2(1) - q1(2) * q2(2) - q1(3) * q2(3);
    crossMultiRes(1) = q1(1) * q2(0) + q1(0) * q2(1) + q1(2) * q2(3) - q1(3) * q2(2);
    crossMultiRes(2) = q1(2) * q2(0) + q1(0) * q2(2) + q1(3) * q2(1) - q1(1) * q2(3);
    crossMultiRes(3) = q1(3) * q2(0) + q1(0) * q2(3) + q1(1) * q2(2) - q1(2) * q2(1);

    return crossMultiRes;
}

// 从欧拉角 v(x, y, z)/v(Roll, Pitch, Yaw) 中获取四元数 Q(q0,q1,q2,q3).
Vector4d Quaternions::GetQFromEuler(Vector3d &euler_angle) const {
    Vector4d eulerQ;

    double r = euler_angle(0) / 2.0;
    double p = euler_angle(1) / 2.0;
    double y = euler_angle(2) / 2.0;


    double sinp = sin(p);
    double siny = sin(y);
    double sinr = sin(r);
    double cosp = cos(p);
    double cosy = cos(y);
    double cosr = cos(r);

    eulerQ(0) = cosr * cosp * cosy + sinr * sinp * siny;
    eulerQ(1) = sinr * cosp * cosy - cosr * sinp * siny;
    eulerQ(2) = cosr * sinp * cosy + sinr * cosp * siny;
    eulerQ(3) = cosr * cosp * siny - sinr * sinp * cosy;

    return eulerQ;
}

// 从四元数获取余弦矩阵DCM
Matrix3d Quaternions::GetDCMFromQ(Vector4d &q) {
    // b系到地理系n系
    Matrix3d dcm_b2n;

    dcm_b2n(0, 0) = q(0) * q(0) + q(1) * q(1) - q(2) * q(2) - q(3) * q(3);
    dcm_b2n(0, 1) = 2 * (q(1) * q(2) - q(0) * q(3));
    dcm_b2n(0, 2) = 2 * (q(1) * q(3) + q(0) * q(2));
    dcm_b2n(1, 0) = 2 * (q(1) * q(2) + q(0) * q(3));
    dcm_b2n(1, 1) = q(0) * q(0) - q(1) * q(1) + q(2) * q(2) - q(3) * q(3);
    dcm_b2n(1, 2) = 2 * (q(2) * q(3) - q(0) * q(1));
    dcm_b2n(2, 0) = 2 * (q(1) * q(3) - q(0) * q(2));
    dcm_b2n(2, 1) = 2 * (q(2) * q(3) + q(0) * q(1));
    dcm_b2n(2, 2) = q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3);

    return dcm_b2n;
}