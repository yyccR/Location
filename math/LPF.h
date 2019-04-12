//
// Created by yangcheng on 2019/2/26.
//

#ifndef LOCATION_LPF_H
#define LOCATION_LPF_H

#include "../system/Status.h"
#include "Eigen/Dense"


class LPF {
public:

    LPF();
    ~LPF();
//    void LowPassFilter();

    void LowPassFilter2ndFactorCal(routing::Status *status);

    Eigen::Vector3d LowPassFilter2nd4ACC(routing::Status *status, Eigen::Vector3d &cur_data);

    Eigen::Vector3d LowPassFilter4Ornt(routing::Status *status, Eigen::Vector3d &ornt_data);

    Eigen::Vector3d JumpPointCompensate(double current, double sec_last, double last);

    std::string JudgeOrientation(double orientation);
};


#endif //LOCATION_LPF_H
