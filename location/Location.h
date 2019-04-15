//
// Created by yangcheng on 2019/1/14.
//


#include "../system/Status.h"
#include "../include/eigen3/Eigen/Dense"

#ifndef LOCATION_LOCATION_H
#define LOCATION_LOCATION_H


class Location {
private:
    // 状态容器
    routing::Status status;

public:

    Location();

    ~Location();

    // 定位,计算当前位置
    void PredictCurrentPosition(Eigen::Vector3d &gyro_data, Eigen::Vector3d &acc_data, Eigen::Vector3d &mag_data,
                                Eigen::VectorXd &gps_data, Eigen::Vector3d &g_data, Eigen::Vector3d &ornt_data,
                                Eigen::Vector2d &road_data
    );

    // 获取当前融合定位的输出
    routing::GNSSINS GetGNSSINS();

    // 获取当前位置
    routing::Position GetCurrentPosition();

    // 获取当前方位角
    double GetCurrentBearing();

    // 设置采样频率
    void SetHz(double f);

    // 采样时间自适应
    void AutoAdjustTFactor(routing::Status *status, Eigen::VectorXd &gps_data, double ins_distance);

    // 方向传感器和GPS方向差值修正
    void UpdateZaxisWithGPSAndRoad(routing::Status *status, Eigen::VectorXd &gps_data, Eigen::Vector3d &ornt_data, Eigen::Vector2d &road_data);

//    Eigen::VectorXd GPSJumpPointCompensate(routing::Status *status, Eigen::VectorXd &gps_bearing_queue);

};


#endif //LOCATION_LOCATION_H
