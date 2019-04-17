//
// Created by yangcheng on 2019/4/17.
//

#ifndef LOCATION_COMPASS_H
#define LOCATION_COMPASS_H

#include <Eigen/Dense>
#include "../system/Status.h"

class Compass {
public:

    bool IsCompassVaild(routing::Status *status, Eigen::Vector3d &ornt_data);
};


#endif //LOCATION_COMPASS_H
