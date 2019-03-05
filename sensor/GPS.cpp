//
// Created by yangcheng on 2018/12/24.
//

#include "GPS.h"
#include "cmath"

using namespace Eigen;
using namespace routing;

/**
 * 给定起点经度,纬度,距离,方向, 计算终点的经纬度.
 *
 * @param startLng 起点经度
 * @param startLat 起点纬度
 * @param distance 距离
 * @param heading 方向（与正北夹角）
 * @return
 */
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

/**
 * 根据GPS速度,方向 计算正北,正东方向速度.
 *
 * @param status
 * @param velocity
 * @param bearing
 */
void GPS::UpdateVelocity(Status *status, double &velocity, double &bearing) {
    double bearing_rad = bearing / 180.0 * M_PI;
    double v_north = velocity * cos(bearing_rad);
    double v_east = velocity * sin(bearing_rad);
    (*status).velocity.v_x = v_north;
    (*status).velocity.v_y = v_east;
}

/**
 * 计算两个经纬度点之间的距离.
 *
 * @param startLng 起点经度
 * @param startLat 起点纬度
 * @param endLng 终点经度
 * @param endLat 终点纬度
 * @return
 */
double GPS::CalDistance(double &startLng, double &startLat, double &endLng, double &endLat) {

    double startLng_rad = startLng / 180.0 * M_PI;
    double startLat_rad = startLat / 180.0 * M_PI;
    double endLng_rad = endLng / 180.0 * M_PI;
    double endLat_rad = endLat / 180.0 * M_PI;


    double a =  pow(sin((endLat_rad - startLat_rad) / 2.0), 2.0) +
                cos(startLat_rad) * cos(endLat_rad) * pow(sin((endLng_rad - startLng_rad) / 2.0), 2.0);
    double c = 2 * asin(sqrt(a));
    return c * 6378.137 * 1000.0;
}