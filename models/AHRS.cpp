//
// Created by yangcheng on 2018/12/26.
//

#include "Eigen/Dense"
#include "../math/Quaternions.cpp"
#include "../sensor/Accelerometer.cpp"
#include "../sensor/Gyroscope.cpp"
#include "../sensor/Magnetometer.cpp"
#include "AHRS.h"

using namespace Eigen;

/**
 * 姿态更新(AHRS: attitude and heading reference system)。
 *
 * @param err: 误差积分指针
 * @param gyro: 陀螺仪数据, 欧拉角
 * @param acc: 加速计数据, 加速度
 * @param mag: 地磁感应数据, 注意手机获取到的磁力计数据单位为 nT, 需乘上 10^-6 转成 mT 单位。
 * @param ki: 比例参数
 * @param kp: 积分参数
 * @param halfT: 采样周期的一半
 * @return 返回更新完的四元数数据
 */
Vector4d AHRS::UpdateAttitude(Vector3d *err, Vector3d &gyro, Vector3d &acc, Vector3d &mag, double &ki, double &kp,
                              double &halfT) const {

    Quaternions quaternions;
    Accelerometer accelerometer;
    Gyroscope gyroscope;
    Magnetometer magnetometer;
    Vector4d newAttitude;

    // 从欧拉角获取四元数
    Vector4d euler_q = quaternions.GetQFromEuler(gyro);
    // 计算旋转矩阵(b系到n系)
    Matrix3d b2n = quaternions.GetDCMFromQ(euler_q);
    Matrix3d n2b = b2n.transpose();

    // 归一化加速计数据和地磁数据
    Vector3d norm_acc = accelerometer.Normalise(acc);
    Vector3d norm_mag = magnetometer.Normalise(mag);

    // 计算加速计误差
    Vector3d rotate_g = accelerometer.RotateG(n2b);
    Vector3d acc_error = accelerometer.GetAccError(norm_acc, rotate_g);

    // 计算地磁感应误差
    Vector3d mag_error = magnetometer.GetMagError(b2n, mag);

    // 计算总误差
    Vector3d e;
    e(0) = acc_error(0) + mag_error(0);
    e(1) = acc_error(1) + mag_error(1);
    e(2) = acc_error(2) + mag_error(2);

    // 误差积分, 累计部分。
    (*err)(0) += e(0) * ki * 2.0 * halfT;
    (*err)(1) += e(1) * ki * 2.0 * halfT;
    (*err)(2) += e(2) * ki * 2.0 * halfT;
    // 误差修正, 比例部分。
    gyro(0) += e(0) * kp + (*err)(0);
    gyro(1) += e(1) * kp + (*err)(1);
    gyro(2) += e(2) * kp + (*err)(2);

    // 重新调整旋转四元数, 一阶龙格库塔法更新四元数
    euler_q(0) = euler_q(0) + (-euler_q(1) * gyro(0) - euler_q(2) * gyro(1) - euler_q(3) * gyro(2)) * halfT;
    euler_q(1) = euler_q(1) + (euler_q(0) * gyro(0) + euler_q(2) * gyro(2) - euler_q(3) * gyro(1)) * halfT;
    euler_q(2) = euler_q(2) + (euler_q(0) * gyro(1) - euler_q(1) * gyro(2) + euler_q(3) * gyro(0)) * halfT;
    euler_q(3) = euler_q(3) + (euler_q(0) * gyro(2) + euler_q(1) * gyro(1) - euler_q(2) * gyro(0)) * halfT;

    newAttitude = quaternions.Normalise(euler_q);
    return newAttitude;

}