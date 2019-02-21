//
// Created by yangcheng on 2018/12/24.
//

#include "GPS.h"
#include "cmath"

using namespace Eigen;

Eigen::Vector2d GPS::CalDestination(double &startLng, double &startLat, double &distance, double &heading) {
    Vector2d nextLngLat;
    double R = 6378.137 * 1000.0;
    double rad_lng = startLng / 180.0 * M_PI;
    double rad_lat = startLat / 180.0 * M_PI;
    double lat = asin(sin(rad_lat) * cos(distance / R) + cos(rad_lat) * sin(distance / R) * cos(heading));
    double lng = rad_lng + atan2(sin(heading) * sin(distance / R) * cos(rad_lat), cos(distance / R) - sin(rad_lat) * sin(lat));
    nextLngLat(0) = lng / M_PI * 180.0;
    nextLngLat(1) = lat / M_PI * 180.0;
    return nextLngLat;
}

void GPS::UpdateVelocity(Status *status, double &velocity, double &bearing) {
    double bearing_rad = bearing / 180.0 * M_PI;
    double v_north = velocity * cos(bearing_rad);
    double v_east = velocity * sin(bearing_rad);
    (*status).velocity.v_x = v_north;
    (*status).velocity.v_y = v_east;
}