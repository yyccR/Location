//
// Created by yangcheng on 2018/12/24.
//

#ifndef LOCATION_GPS_H
#define LOCATION_GPS_H

#include "Eigen/Dense"
#include "../system/Status.h"

class GPS {
public:

    // 给定起点经度,纬度,距离,方向, 计算终点的经纬度.
    Eigen::Vector2d CalDestination(double &startLng, double &startLat, double &distance, double &heading);

    // 利用GPS速度作为加速计初始速度,根据GPS速度,方向 计算正北,正东方向速度.
    void UpdateVelocity(routing::Status *status, double &velocity, double &bearing);

    // 计算两个经纬度点之间的距离
    double CalDistance(double &startLng, double &startLat, double &endLng, double &endLat);

    // 根据当前输入以及状态判断采用GPS还是INS
    bool IsGPSValid(routing::Status *status,  Eigen::VectorXd &gps_data);
};


#endif //LOCATION_GPS_H
