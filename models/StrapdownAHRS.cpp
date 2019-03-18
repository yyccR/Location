//
// Created by yangcheng on 2019/3/13.
//

#include "StrapdownAHRS.h"
#include "../math/Quaternions.h"

using namespace Eigen;
using namespace routing;

/**
 * 捷联式姿态更新
 *
 * @param q_attitude, 上衣时刻姿态四元数
 * @param gyro, 陀螺仪旋转角
 * @param acc, 加速计输出
 * @return
 */
Vector4d StrapdownAHRS::StrapdownUpdateAttitude(Vector4d &q_attitude, Vector3d &gyro,  Status *status) {

    // 四元数转方向余弦矩阵
    Quaternions quaternions;
    Matrix3d dcm_b2n = quaternions.GetDCMFromQ(q_attitude);


    /**
     * dcm_b2n(t+1) = dcm_b2n(t) * σ
     * //Ak = I + sinσ/σ * (σ X) + (1 - cosσ)/σ² * (σ X)²
     * σ = wn_b * deltaT = (w_i_b - dcm_n2b(t) * (wi_n + we_n)) * deltaT
     */

    // 导航坐标系下的地球自转角速度
    double cur_lat = (*status).position.lat * M_PI / 180.0;
    Vector3d wi_n((*status).parameters.we * cos(cur_lat), 0.0, -(*status).parameters.we * sin(cur_lat));
    // 导航坐标系下的旋转速率
    double v_east = (*status).velocity.v_y;
    double v_north = (*status).velocity.v_x;
    double Rh = (*status).parameters.R + (*status).position.altitude;
    Vector3d we_n(v_east / Rh, -v_north / Rh, -v_east * tan(cur_lat) / Rh);
    // 计算载体相对导航系的角速度
    Matrix3d dcm_n2b = dcm_b2n.transpose();
    Vector3d wn_b = gyro - dcm_n2b * (wi_n + we_n);
//    Vector3d wn_b = gyro;

    // 计算载体相对导航系角速率斜对称阵
    Matrix3d Ob_n;
    Ob_n(0,0) = 0.0;
    Ob_n(0,1) = -wn_b(2) * (*status).parameters.t;
    Ob_n(0,2) = wn_b(1) * (*status).parameters.t;
    Ob_n(1,0) = wn_b(2) * (*status).parameters.t;
    Ob_n(1,1) = 0.0;
    Ob_n(1,2) = -wn_b(0) * (*status).parameters.t;
    Ob_n(2,0) = -wn_b(1) * (*status).parameters.t;
    Ob_n(2,1) = wn_b(0) * (*status).parameters.t;
    Ob_n(2,2) = 0.0;
    // 计算载体相对导航系角速率斜对称阵平方
//    Matrix3d Ob_n2;
//    Vector3d wn_bt = (*status).parameters.t * wn_b;
//    Ob_n2(0,0) = - (wn_bt(1) * wn_bt(1) + wn_bt(2) * wn_bt(2));
//    Ob_n2(0,1) = wn_bt(0) * wn_b(1);
//    Ob_n2(0,2) = wn_bt(0) * wn_b(2);
//    Ob_n2(1,0) = wn_bt(0) * wn_b(1);
//    Ob_n2(1,1) = - (wn_bt(0) * wn_bt(0) + wn_bt(2) * wn_bt(2));
//    Ob_n2(1,2) = wn_bt(1) * wn_b(2);
//    Ob_n2(2,0) = wn_bt(0) * wn_b(2);
//    Ob_n2(2,1) =  wn_bt(1) * wn_b(2);
//    Ob_n2(2,2) =  - (wn_bt(0) * wn_bt(0) + wn_bt(1) * wn_bt(1));
//    // 计算更新矩阵Ak
//    double sigma = wn_bt.norm();
//    Matrix3d Ak = MatrixXd::Identity(3,3) + sin(sigma)/sigma * Ob_n + (1 - cos(sigma)/(sigma*sigma)) * Ob_n2;

    // 更新姿态
    Matrix3d dcm_b2n_new = dcm_b2n * Ob_n;
    Vector4d q_attitude_new = quaternions.GetQfromDCM(dcm_b2n_new);
    Vector4d q_attitude_new_norm = quaternions.Normalise(q_attitude_new);
    return q_attitude_new_norm;
}