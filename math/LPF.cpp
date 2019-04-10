//
// Created by yangcheng on 2019/2/26.
//

#include "LPF.h"
#include "iostream"

using namespace routing;
using namespace Eigen;

/**
 * 二级低通滤波参数计算
 * @param status
 */
void LPF::LowPassFilter2ndFactorCal(Status *status) {
    double a = 1 / (2 * M_PI * status->parameters.acc_hz * status->parameters.t);
    status->parameters.acc_b0 = 1 / (a*a + 3*a + 1);
    status->parameters.acc_a1 = (2*a*a + 3*a) / (a*a + 3*a + 1);
    status->parameters.acc_a2 = (a*a) / (a*a + 3*a + 1);


    double o = 1 / (2 * M_PI * status->parameters.ornt_hz * status->parameters.t);
    status->parameters.ornt_b0 = 1 / (o*o + 3*o + 1);
    status->parameters.ornt_a1 = (2*o*o + 3*o) / (o*o + 3*o + 1);
    status->parameters.ornt_a2 = (o*o) / (o*o + 3*o + 1);
}

/**
 * 二阶低通滤波
 *
 * @param status
 * @param cur_data
 */
Vector3d LPF::LowPassFilter2nd4ACC(Status *status, Vector3d &cur_data) {
    Vector3d lpf_acc;

    double b0 = status->parameters.acc_b0;
    double a1 = status->parameters.acc_a1;
    double a2 = status->parameters.acc_a2;
    lpf_acc(0) = cur_data(0) * b0 + status->parameters.sec_last_acc_data(0) * a1 - status->parameters.last_acc_data(0) * a2;
    lpf_acc(1) = cur_data(1) * b0 + status->parameters.sec_last_acc_data(1) * a1 - status->parameters.last_acc_data(1) * a2;
    lpf_acc(2) = cur_data(2) * b0 + status->parameters.sec_last_acc_data(2) * a1 - status->parameters.last_acc_data(2) * a2;

    status->parameters.last_acc_data = status->parameters.sec_last_acc_data;
    status->parameters.sec_last_acc_data = lpf_acc;
//    std::cout << "parameter " << b0 << " " <<  a1  << " " << a2 << std::endl;
    return lpf_acc;
}

Eigen::Vector3d LPF::LowPassFilter4Ornt(routing::Status *status, Eigen::Vector3d &ornt_data) {
    Vector3d lpf_ornt;

    double b0 = status->parameters.ornt_b0;
    double a1 = status->parameters.ornt_a1;
    double a2 = status->parameters.ornt_a2;
    lpf_ornt(0) = ornt_data(0) * b0 + status->parameters.sec_last_ornt_data(0) * a1 - status->parameters.last_ornt_data(0) * a2;
    lpf_ornt(1) = ornt_data(1) * b0 + status->parameters.sec_last_ornt_data(1) * a1 - status->parameters.last_ornt_data(1) * a2;
    lpf_ornt(2) = ornt_data(2) * b0 + status->parameters.sec_last_ornt_data(2) * a1 - status->parameters.last_ornt_data(2) * a2;

    status->parameters.last_ornt_data = status->parameters.sec_last_ornt_data;
    status->parameters.sec_last_ornt_data = lpf_ornt;
//    std::cout << ornt_data(2) << " " << lpf_ornt(2) << std::endl;
//    std::cout << "parameter " << b0 << " " <<  a1  << " " << a2 << std::endl;
    return lpf_ornt;

}

// 初始化,计算对应参数
LPF::LPF() {
//    LowPassFilter2ndFactorCal(status);
}

LPF::~LPF() {}