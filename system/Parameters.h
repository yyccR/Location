//
// Created by yangcheng on 2019/1/13.
//

#include "Eigen/Dense";

#ifndef LOCATION_PARAMETERS_H
#define LOCATION_PARAMETERS_H

using namespace Eigen;

struct Parameters{

    // 陀螺仪标定参数, v(offset_x,offset_y,offset_z)
    VectorXd gyro_coef;

    // 加速计标定参数, v(offset_x,offset_y,offset_z,scale_x,scale_y,scale_z)
    VectorXd acc_coef;

    // 地磁计标定参数, v(offset_x,offset_y,offset_z,scale_x,scale_y,scale_z)
    VectorXd mag_coef;

    // LM算法标定参数, 对加速计和地磁计通用
    // 初始化阻尼因子用, mu = gamma * max(A), A = Jacobi_t * Jacobi;
    double gamma;
    // 迭代精度
    double epsilon;
    // 最高迭代次数
    int max_step;

};



#endif //LOCATION_PARAMETERS_H
