//
// Created by yangcheng on 2019/2/26.
//

#ifndef LOCATION_LPF_H
#define LOCATION_LPF_H

#include "../system/Status.h"
#include "Eigen/Dense"


class LPF {
public:

    LPF(routing::Status *status);
    ~LPF();
//    void LowPassFilter();

    void LowPassFilter2ndFactorCal(routing::Status *status);

    Eigen::Vector3d LowPassFilter2nd(routing::Status *status, Eigen::Vector3d &cur_data);
};


#endif //LOCATION_LPF_H
