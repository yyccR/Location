//
// Created by yangcheng on 2018/12/24.
//

#include "cmath"
#include "Magnetometer.h"
#include "../math/Optimizer.h"

using namespace Eigen;
using namespace routing;

Vector3d Magnetometer::Normalise(Vector3d &m) const {
    Vector3d normM;
    double norm2 = m(0) * m(0) + m(1) * m(1) + m(2) * m(2);
    // 如果四元数各项足够接近单位四元数, 则不做任何处理.
    if (norm2 != 0.0) {
        double norm = sqrt(norm2);
        normM(0) = m(0) / norm;
        normM(1) = m(1) / norm;
        normM(2) = m(2) / norm;
    } else {
        normM = m;
    }
    return normM;
}


Vector3d Magnetometer::GetMagError(Vector4d &q, Vector3d &originMag) {
    double hx = 2.0 * (originMag(0) * (0.5 - q(2) * q(2) - q(3) * q(3)) + originMag(1) * (q(1) * q(2) - q(0) * q(3)) +
                       originMag(2) * (q(1) * q(3) + q(0) * q(2)));
    double hy = 2.0 * (originMag(0) * (q(1) * q(2) + q(0) * q(3)) + originMag(1) * (0.5 - q(1) * q(1) - q(3) * q(3)) +
                       originMag(2) * (q(2) * q(3) - q(0) * q(1)));
    double bx = sqrt(hx * hx + hy * hy);
    double bz = 2.0f * (originMag(0) * (q(1) * q(3) - q(0) * q(2)) + originMag(1) * (q(2) * q(3) + q(0) * q(1)) +
                        originMag(2) * (0.5 - q(1) * q(1) - q(2) * q(2)));


    double halfwx = bx * (0.5 - q(2) * q(2) - q(3) * q(3)) + bz * (q(1) * q(3) - q(0) * q(2));
    double halfwy = bx * (q(1) * q(2) - q(0) * q(3)) + bz * (q(0) * q(1) + q(2) * q(3));
    double halfwz = bx * (q(0) * q(2) + q(1) * q(3)) + bz * (0.5f - q(1) * q(1) - q(2) * q(2));

    double ex = originMag(1) * halfwz - originMag(2) * halfwy;
    double ey = originMag(2) * halfwx - originMag(0) * halfwz;
    double ez = originMag(0) * halfwy - originMag(1) * halfwx;
    Vector3d e(ex, ey, ez);

    return e;
}

// 地磁感应误差计算
Vector3d Magnetometer::GetMagError(Matrix3d &b2n, Vector3d &originMag) const {

    Vector3d magErr;
    Vector3d formatMag;

    // 旋转b系下地磁数据到n系.
    Vector3d rotatedMag = b2n * originMag;
    // 地理坐标系下的真实地磁数据。
    formatMag(0) = sqrt(rotatedMag(0) * rotatedMag(0) + rotatedMag(1) * rotatedMag(1));
    formatMag(1) = 0.0;
    formatMag(2) = rotatedMag(2);
    // 转到b系
    Vector3d realMag = b2n.transpose() * formatMag;

    // 计算误差.
//    magErr(0) = realMag(1) * originMag(2) - realMag(2) * originMag(1);
//    magErr(1) = realMag(2) * originMag(0) - realMag(0) * originMag(2);
//    magErr(2) = realMag(0) * originMag(1) - realMag(1) * originMag(0);

    magErr(0) = realMag(2) * originMag(1) - realMag(1) * originMag(2);
    magErr(1) = realMag(0) * originMag(2) - realMag(2) * originMag(0);
    magErr(2) = realMag(1) * originMag(0) - realMag(0) * originMag(1);

    return magErr;
}

void Magnetometer::MagCalibration(MatrixXd &input_data, Status *status) {
    double mag = (*status).parameters.mag;
    double gamma = (*status).parameters.gamma;
    double epsilon = (*status).parameters.epsilon;
    int max_step = (*status).parameters.max_step;
    VectorXd *coef = &(*status).parameters.mag_coef;
    Optimizer optimizer;
    optimizer.LevenbergMarquardt(input_data, mag, coef, gamma, epsilon, max_step);
}