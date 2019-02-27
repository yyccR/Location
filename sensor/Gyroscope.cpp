//
// Created by yangcheng on 2018/12/13.
//

#include "Gyroscope.h"
#include <Eigen/Dense>
#include <cmath>

using namespace Eigen;

///**
// * 从陀螺仪获取姿态旋转矩阵(方向余弦矩阵DCM), b系坐标转g系
// *
// * @param gyro, 陀螺仪欧拉角w(x,y,z);
// * @param deltaT
// * @return 方向余弦矩阵DCM
// */
//Matrix3d Gyroscope::GetDCM(Vector3d &gyro, double &deltaT) {
//
//    // 计算deltaT时间内角度变化值，deltaT越小越精确。
//    double delta_wx = gyro(0) * deltaT;
//    double delta_wy = gyro(1) * deltaT;
//    double delta_wz = gyro(2) * deltaT;
//
//    double cos_delta_wx = cos(delta_wx);
//    double sin_delta_wx = sin(delta_wx);
//    double cos_delta_wy = cos(delta_wy);
//    double sin_delta_wy = sin(delta_wy);
//    double cos_delta_wz = cos(delta_wz);
//    double sin_delta_wz = sin(delta_wz);
//
//    // 计算DCM矩阵(z * y * x)。
//    Matrix3d DCM;
//    DCM(0, 0) = cos_delta_wy * cos_delta_wz;
//    DCM(0, 1) = -sin_delta_wz * cos_delta_wx + sin_delta_wx * sin_delta_wy * cos_delta_wz;
//    DCM(0, 2) = sin_delta_wx * sin_delta_wz + sin_delta_wy * cos_delta_wx * cos_delta_wz;
//    DCM(1, 0) = sin_delta_wz * cos_delta_wy;
//    DCM(1, 1) = cos_delta_wx * cos_delta_wz + sin_delta_wx * sin_delta_wy * sin_delta_wz;
//    DCM(1, 2) = -sin_delta_wx * cos_delta_wz + sin_delta_wy * sin_delta_wz * cos_delta_wx;
//    DCM(2, 0) = -sin_delta_wy;
//    DCM(2, 1) = sin_delta_wx * cos_delta_wy;
//    DCM(2, 2) = cos_delta_wx * cos_delta_wy;
//
//    return DCM;
//}

/**
 * 陀螺仪标定,测量零飘误差.
 *
 * @param input_data, 静止时的陀螺仪数据(建议,n=200);
 * @param parameters , 陀螺仪标定参数 coef(offset_x,offset_y,offset_z);
 */
void Gyroscope::GyroCalibration(MatrixXd &input_data, Status *status) {

    int data_nums = static_cast<int>(input_data.rows());
    double offset_x = 0;
    double offset_y = 0;
    double offset_z = 0;

    // 用于判断陀螺仪是否静止, 若非静止, 标定将会产生较大误差.
    double x_diff = 0;
    double y_diff = 0;
    double z_diff = 0;
    double gyro_x_init = input_data(0, 0);
    double gyro_y_init = input_data(0, 1);
    double gyro_z_init = input_data(0, 2);

    for (int i = 0; i < data_nums; i++) {

        // 计算前后偏差,用以判断是否处于静止.
        x_diff += input_data(i, 0) - gyro_x_init;
        y_diff += input_data(i, 1) - gyro_y_init;
        z_diff += input_data(i, 2) - gyro_z_init;
        // 重赋x,y,z, 用于计算两两样本前后差
        gyro_x_init = input_data(i, 0);
        gyro_y_init = input_data(i, 1);
        gyro_z_init = input_data(i, 2);
        // 计算offset
        offset_x += input_data(i, 0);
        offset_y += input_data(i, 1);
        offset_z += input_data(i, 2);
    }

    // 判断是否静止
    if (x_diff < data_nums * 0.5 && y_diff < data_nums * 0.5 && z_diff < data_nums * 0.5) {
        VectorXd gyro_coef(3);
        gyro_coef(0) = offset_x / data_nums;
        gyro_coef(1) = offset_y / data_nums;
        gyro_coef(2) = offset_z / data_nums;

        (*status).parameters.gyro_coef = gyro_coef;
    }
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