//
// Created by yangcheng on 2019/2/26.
//

#include "LPF.h"
#include "iostream"
#include <string.h>

using namespace routing;
using namespace Eigen;

/**
 * 二级低通滤波参数计算
 * @param status
 */
void LPF::LowPassFilter2ndFactorCal(Status *status) {
    double a = 1 / (2 * M_PI * status->parameters.acc_hz * status->parameters.t);
    status->parameters.acc_b0 = 1 / (a * a + 3 * a + 1);
    status->parameters.acc_a1 = (2 * a * a + 3 * a) / (a * a + 3 * a + 1);
    status->parameters.acc_a2 = (a * a) / (a * a + 3 * a + 1);


    double o = 1 / (2 * M_PI * status->parameters.ornt_hz * status->parameters.t);
    status->parameters.ornt_b0 = 1 / (o * o + 3 * o + 1);
    status->parameters.ornt_a1 = (2 * o * o + 3 * o) / (o * o + 3 * o + 1);
    status->parameters.ornt_a2 = (o * o) / (o * o + 3 * o + 1);
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
    lpf_acc(0) =
            cur_data(0) * b0 + status->parameters.sec_last_acc_data(0) * a1 - status->parameters.last_acc_data(0) * a2;
    lpf_acc(1) =
            cur_data(1) * b0 + status->parameters.sec_last_acc_data(1) * a1 - status->parameters.last_acc_data(1) * a2;
    lpf_acc(2) =
            cur_data(2) * b0 + status->parameters.sec_last_acc_data(2) * a1 - status->parameters.last_acc_data(2) * a2;

    status->parameters.last_acc_data = status->parameters.sec_last_acc_data;
    status->parameters.sec_last_acc_data = lpf_acc;
//    std::cout << "parameter " << b0 << " " <<  a1  << " " << a2 << std::endl;
    return lpf_acc;
}


std::string LPF::JudgeOrientation(double orientation) {
    if (orientation <= 90.0) {
        return "1";
    } else if (orientation > 90.0 && orientation <= 180.0) {
        return "2";
    } else if (orientation > 180.0 && orientation <= 270.0) {
        return "3";
    } else {
        return "4";
    }
}

/**
 * 指南针360度跳点补偿.
 *
 * @param current
 * @param sec_last
 * @param last
 * @return
 */
Eigen::Vector3d LPF::JumpPointCompensate(double current, double sec_last, double last) {
    Vector3d lpf_compensate;
    std::string flag = JudgeOrientation(current) + JudgeOrientation(sec_last) + JudgeOrientation(last);

    if (flag == "144") {
        lpf_compensate(0) = current + 360.0;
        lpf_compensate(1) = sec_last;
        lpf_compensate(2) = last;
    } else if (flag == "114") {
        lpf_compensate(0) = current + 360.0;
        lpf_compensate(1) = sec_last + 360.0;
        lpf_compensate(2) = last;
    } else if (flag == "141") {
        lpf_compensate(0) = current + 360;
        lpf_compensate(1) = sec_last;
        lpf_compensate(2) = last;
    } else if (flag == "411") {
        lpf_compensate(0) = current;
        lpf_compensate(1) = sec_last + 360.0;
        lpf_compensate(2) = last + 360.0;
    } else if (flag == "441") {
        lpf_compensate(0) = current;
        lpf_compensate(1) = sec_last;
        lpf_compensate(2) = last + 360.0;
    } else if (flag == "414") {
        lpf_compensate(0) = current;
        lpf_compensate(1) = sec_last + 360.0;
        lpf_compensate(2) = last;
    } else {
        lpf_compensate(0) = current;
        lpf_compensate(1) = sec_last;
        lpf_compensate(2) = last;
    }
    return lpf_compensate;
}

Eigen::Vector3d LPF::LowPassFilter4Ornt(routing::Status *status, Eigen::Vector3d &ornt_data) {
    Vector3d lpf_ornt;

    double b0 = status->parameters.ornt_b0;
    double a1 = status->parameters.ornt_a1;
    double a2 = status->parameters.ornt_a2;
    for(int i = 0; i < 3; ++i){
        Vector3d lpf_compensate = JumpPointCompensate(ornt_data(i), status->parameters.sec_last_ornt_data(i), status->parameters.last_ornt_data(i));
        double lpf_filter = lpf_compensate(0) * b0 + lpf_compensate(1) * a1 - lpf_compensate(2) * a2;
        if(lpf_filter > 360.0){
            lpf_ornt(i) = lpf_filter - 360.0;
        }else{
            lpf_ornt(i) = lpf_filter;
        }
    }

//    lpf_ornt(0) = ornt_data(0) * b0 + status->parameters.sec_last_ornt_data(0) * a1 - status->parameters.last_ornt_data(0) * a2;
//    lpf_ornt(1) = ornt_data(1) * b0 + status->parameters.sec_last_ornt_data(1) * a1 - status->parameters.last_ornt_data(1) * a2;
//    lpf_ornt(2) = ornt_data(2) * b0 + status->parameters.sec_last_ornt_data(2) * a1 - status->parameters.last_ornt_data(2) * a2;

    status->parameters.last_ornt_data = status->parameters.sec_last_ornt_data;
    status->parameters.sec_last_ornt_data = lpf_ornt;
    std::cout << ornt_data(2) << " " << lpf_ornt(2) << std::endl;
//    std::cout << "parameter " << b0 << " " <<  a1  << " " << a2 << std::endl;
    return lpf_ornt;

}

// 初始化,计算对应参数
LPF::LPF() {
//    LowPassFilter2ndFactorCal(status);
}

LPF::~LPF() {}