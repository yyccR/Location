//
// Created by yangcheng on 2018/12/24.
//

#ifndef LOCATION_GPS_H
#define LOCATION_GPS_H

#include "Eigen/Dense"

class GPS {
public:

    Eigen::Vector2d CalDestination(double &startLng, double &startLat, double distance, double heading);
};


#endif //LOCATION_GPS_H
