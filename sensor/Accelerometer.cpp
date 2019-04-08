//
// Created by yangcheng on 2018/11/28.
//

#include "Accelerometer.h"
#include "../math/Optimizer.h"
#include "iostream"
#include "../math/LPF.h"

using namespace Eigen;
using namespace routing;

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

Vector3d Accelerometer::GetAccError(Vector3d &originA, Vector4d &q) const {
    double halfvx = q(1) * q(3) - q(0) * q(2);
    double halfvy = q(0) * q(1) + q(2) * q(3);
    double halfvz = q(0) * q(0) - 0.5 + q(3) * q(3);

    double halfex = originA(1) * halfvz - originA(2) * halfvy;
    double halfey = originA(2) * halfvx - originA(0) * halfvz;
    double halfez = originA(0) * halfvy - originA(1) * halfvx;
    Vector3d e(halfex,halfey,halfez);
    return e;
}

// 加速计向量(originA)叉乘地理重力转b系(rotatedG)误差，用于较正陀螺仪.
Vector3d Accelerometer::GetAccError(Vector3d &originA, Vector3d &rotatedG) const {
    Vector3d accErr;

    accErr(0) = originA(1) * rotatedG(2) - originA(2) * rotatedG(1);
    accErr(1) = originA(2) * rotatedG(0) - originA(0) * rotatedG(2);
    accErr(2) = originA(0) * rotatedG(1) - originA(1) * rotatedG(0);

    return accErr;
}

void Accelerometer::AccCalibration(MatrixXd &input_data, Status *status) {
    double R = 1.0;
    double gamma = (*status).parameters.gamma;
    double epsilon = (*status).parameters.epsilon;
    int max_step = (*status).parameters.max_step;
    VectorXd *coef = &(*status).parameters.acc_coef;
    MatrixXd input_data_format = input_data / (*status).parameters.g;
    Optimizer optimizer;
    optimizer.LevenbergMarquardt(input_data_format, R, coef, gamma, epsilon, max_step);
}


void Accelerometer::PositionIntegral(Status *status, Vector3d &acc, double t) {

    // 更新位置
    (*status).position.x += (*status).velocity.v_x * t + 0.5 * acc(0) * t * t;
    (*status).position.y += (*status).velocity.v_y * t + 0.5 * acc(1) * t * t;
    (*status).position.z += (*status).velocity.v_z * t + 0.5 * acc(2) * t * t;
    // 更新速度
    (*status).velocity.v_x = (*status).velocity.v_x + acc(0) * t;
    (*status).velocity.v_y = (*status).velocity.v_y + acc(1) * t;
    (*status).velocity.v_z = (*status).velocity.v_z + acc(2) * t;

}

/**
 * 捷联更新位置速度
 *
 * @param status
 * @param acc, 载体系加速度
 * @param q_attitude, 上一时刻姿态四元数
 */
void Accelerometer::StrapdownUpdateVelocityPosition(Status *status, Vector3d &acc, Vector4d &q_attitude, Vector3d &g_data) {

    Quaternions quaternions;

    /**
     * ve_n_new = acc_n - (2*wi_n + we_n) X ve_n - gl_n;
     */

    // 计算导航系加速度
    Matrix3d dcm_b2n = quaternions.GetDCMFromQ(q_attitude);
    Vector3d acc_n = dcm_b2n * acc;

    // 导航坐标系下的地球自转角速度
    double cur_lat = (*status).position.lat * M_PI / 180.0;
    Vector3d wi_n((*status).parameters.we * cos(cur_lat), 0.0, -(*status).parameters.we * sin(cur_lat));
    // 导航坐标系下的旋转速率
    double v_east = (*status).velocity.v_y;
    double v_north = (*status).velocity.v_x;
    double v_down = (*status).velocity.v_z;
    Vector3d ve_n(v_north, v_east, v_down);
    double Rh = (*status).parameters.R + (*status).position.altitude;
    Vector3d we_n(v_east / Rh, -v_north / Rh, -v_east * tan(cur_lat) / Rh);

    // 计算导航系下的重力加速计修正向量
//    Vector3d gn(0.0,0.0,(*status).parameters.g);
    Vector3d gn = dcm_b2n * g_data;
    double beta = (*status).parameters.we * (*status).parameters.we * Rh / 2.0;
    Vector3d wwR(beta * sin(2.0 * cur_lat), 0.0, beta * (1.0 + cos(2.0 * cur_lat)));
    Vector3d gl_n = gn - wwR;

    // 计算导航系下加速度减去地球旋转影响和重力影响
    Vector3d acc_n_not_filter = acc_n - (2.0 * wi_n + we_n).cross(ve_n) - gl_n;

//    std::cout << "acc_n_not_filter " << acc_n_not_filter.transpose() << std::endl;
    Vector3d acc_n_real = FilterData(status, acc_n_not_filter);
//    std::cout << "acc_n_real " << acc_n_real.transpose() << std::endl;

    // 速度和位置积分
    double deltaT = (*status).parameters.t;
    double v_x_new = v_north + acc_n_real(0) * deltaT;
    double v_y_new = v_east + acc_n_real(1) * deltaT;
    double v_z_new = v_down + acc_n_real(2) * deltaT;
    double x_new = (*status).position.x + (v_north + v_x_new) * deltaT * (*status).parameters.move_t_factor * 0.5;
    double y_new = (*status).position.y + (v_east + v_y_new) * deltaT * (*status).parameters.move_t_factor * 0.5;
    double z_new = (*status).position.z + (v_down + v_z_new) * deltaT * (*status).parameters.move_t_factor * 0.5;
//    double x_new = (v_north + v_x_new) * deltaT * 0.5;
//    double y_new = (v_east + v_y_new) * deltaT * 0.5;
//    double z_new = (v_down + v_z_new) * deltaT * 0.5;
//    std::cout.precision(9);
//    std::cout << x_new << " " << y_new << std::endl;
//    std::cout << "acc " << acc_n_real.transpose() << std::endl;
//    std::cout << "v " << v_x_new << " " << v_y_new << " " << v_z_new  <<std::endl;

    // 更新速度和位置
    (*status).velocity.v_x = v_x_new;
    (*status).velocity.v_y = v_y_new;
    (*status).velocity.v_z = v_z_new;
    (*status).position.x = x_new;
    (*status).position.y = y_new;
    (*status).position.z = z_new;
}

/**
 * 加速计数据过滤
 *
 * @param status
 * @param acc_data
 * @return
 */
Vector3d Accelerometer::FilterData(Status *status, Vector3d &acc_data) {
    LPF lpf;
    Vector3d filter_acc;
    filter_acc = lpf.LowPassFilter2nd(status, acc_data);
    if(abs(filter_acc(0)) <= (*status).parameters.acc_thres){
        filter_acc(0) = 0.0;
    }
    if(abs(filter_acc(1)) <= (*status).parameters.acc_thres){
        filter_acc(1) = 0.0;
    }
    if(abs(filter_acc(2)) <= (*status).parameters.acc_thres){
        filter_acc(2) = 0.0;
    }
    return  filter_acc;
}

Accelerometer::Accelerometer() {}

Accelerometer::~Accelerometer() = default;
