//
// Created by yangcheng on 2018/12/26.
//

#include "Eigen/Dense"
#include "../math/Quaternions.h"
#include "../sensor/Accelerometer.h"
#include "../sensor/Magnetometer.h"
#include "AHRS.h"
#include "iostream"

using namespace Eigen;

/**
 * 姿态更新(AHRS: attitude and heading reference system)。
 *
 * @param err: 误差积分
 * @param gyro: 陀螺仪数据, 欧拉角
 * @param acc: 加速计数据, 该数据需为重力传感器数据 或者 静止时的加速计数据。
 * @param mag: 地磁感应数据, 注意手机获取到的磁力计数据单位为 μT, 需乘上 10^-3 转成 mT 单位。
 * @param ki: 比例参数
 * @param kp: 积分参数
 * @param halfT: 采样周期的一半
 * @return 返回更新完的四元数数据
 */
Vector4d AHRS::UpdateAttitude(Vector3d *err, Vector4d &q_attitude, Vector3d &gyro, Vector3d &acc, Vector3d &mag, double &ki, double &kp,
                              double &halfT) const {

    Quaternions quaternions;
    Accelerometer accelerometer;
    Magnetometer magnetometer;
    Vector4d newAttitude;

    // 从欧拉角获取四元数
//    Vector4d euler_q = quaternions.GetQFromEuler(orientation);
    Vector4d euler_q = q_attitude;

    // 计算旋转矩阵(b系到n系)
    Matrix3d b2n = quaternions.GetDCMFromQ(euler_q);
    Matrix3d n2b = b2n.transpose();

    // 归一化加速计数据和地磁数据
    Vector3d norm_acc = accelerometer.Normalise(acc);
    Vector3d norm_mag = magnetometer.Normalise(mag);

    // 计算加速计误差
//    Vector3d rotate_g = accelerometer.RotateG(n2b);
//    std::cout << rotate_g.transpose() << std::endl;
//    Vector3d acc_error = accelerometer.GetAccError(norm_acc, rotate_g);
    Vector3d acc_error = accelerometer.GetAccError(norm_acc, euler_q);

    // 计算地磁感应误差
//    Vector3d mag_error = magnetometer.GetMagError(b2n, mag);
    Vector3d mag_error = magnetometer.GetMagError(euler_q, mag);

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

    // Integrate rate of change of quaternion
    gyro(0) *= halfT;		// pre-multiply common factors
    gyro(1) *= halfT;
    gyro(2) *= halfT;
    double qa = euler_q(0);
    double qb = euler_q(1);
    double qc = euler_q(2);
    euler_q(0) += (-qb * gyro(0) - qc * gyro(1) - euler_q(3) * gyro(2));
    euler_q(1) += (qa * gyro(0) + qc * gyro(2) - euler_q(3) * gyro(1));
    euler_q(2) += (qa * gyro(1) - qb * gyro(2) + euler_q(3) * gyro(0));
    euler_q(3) += (qa * gyro(2) + qb * gyro(1) - qc * gyro(0));

    // 重新调整旋转四元数, 一阶龙格库塔法更新四元数
//    euler_q(0) = euler_q(0) + (-euler_q(1) * gyro(0) - euler_q(2) * gyro(1) - euler_q(3) * gyro(2)) * halfT;
//    euler_q(1) = euler_q(1) + (euler_q(0) * gyro(0) + euler_q(2) * gyro(2) - euler_q(3) * gyro(1)) * halfT;
//    euler_q(2) = euler_q(2) + (euler_q(0) * gyro(1) - euler_q(1) * gyro(2) + euler_q(3) * gyro(0)) * halfT;
//    euler_q(3) = euler_q(3) + (euler_q(0) * gyro(2) + euler_q(1) * gyro(1) - euler_q(2) * gyro(0)) * halfT;


    newAttitude = quaternions.Normalise(euler_q);
//    Vector3d eur = quaternions.GetEulerFromQ(newAttitude) * 180.0 / M_PI ;
//    std::cout << eur.transpose() << std::endl;
    return newAttitude;

}