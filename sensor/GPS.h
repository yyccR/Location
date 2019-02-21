//
// Created by yangcheng on 2018/12/24.
//

#ifndef LOCATION_GPS_H
#define LOCATION_GPS_H

#include "Eigen/Dense"
#include "../system/Status.h"

class GPS {
public:

    Eigen::Vector2d CalDestination(double &startLng, double &startLat, double &distance, double &heading);

    // 利用GPS速度作为加速计初始速度
    void UpdateVelocity(Status *status, double &velocity, double &bearing);
};


#endif //LOCATION_GPS_H
