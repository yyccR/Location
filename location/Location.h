//
// Created by yangcheng on 2019/1/14.
//

#ifndef LOCATION_LOCATION_H
#define LOCATION_LOCATION_H

#include "../system/Status.h"
#include "../include/eigen3/Eigen/Dense"
#include "../models/StopDetection.h"
#include <memory>

using LOG_CALLBACK = std::function<void(std::string)>;

class Location {

private:

    // 状态容器
    routing::Status status;
    LOG_CALLBACK Log;

    // 无信号时检测行车状态的模型
    std::shared_ptr<StopDetection> stopDetection;

    // 判断模型文件是否存在或损坏
    bool IsFileVaild(std::string &model_path);
    // 加载模型
    void LoadStopDetectModel();
    // 预测整个过程中的行车状态0/1
    void PredictStopStatus(Eigen::Vector3d &gyro_data, Eigen::Vector3d &acc_data, Eigen::Vector3d &mag_data,
                           Eigen::Vector3d &g_data, Eigen::Vector3d &ornt_data);

    // 获取当前位置
    routing::Position GetCurrentPosition();

    // 获取当前方位角
    double GetCurrentBearing();

    // 采样时间自适应
    void AutoAdjustTFactor(Eigen::VectorXd &gps_data, double ins_distance);

    // 惯导运动衰减因子
    void AutoAdjustMovingFactor();

    // 方向传感器和GPS方向差值修正
    void UpdateZaxisWithGPSAndRoad(Eigen::VectorXd &gps_data, Eigen::Vector3d &ornt_data,
                                   Eigen::Vector3d &road_data);

    // 方向传感器和GPS方向差值修正
    void UpdateZaxisWithGPS(Eigen::VectorXd &gps_data, Eigen::Vector3d &ornt_data);

    // 方向传感器和道路方向差值修正
    void UpdateZaxisWithRoad(Eigen::Vector3d &ornt_data, Eigen::Vector3d &road_data);

    // 判断道路方向变化幅度是否与指南针幅度一致
    bool IsRoadCompassSameRange(Eigen::Vector3d &ornt_data, Eigen::Vector3d &road_data);

    // 判断当前是否正处于重新规划中
    bool IsRouting(Eigen::Vector3d &ornt_data, Eigen::Vector3d &road_data);

    // 判断是否偏离航道
    bool IsOffCourse(Eigen::Vector3d &ornt_data, Eigen::Vector3d &road_data);

    // 更新道路类型
    void UpdateRoadType(Eigen::Vector3d &road_data);

public:

    Location();

    ~Location();
    
    void SetLogCallback(LOG_CALLBACK callback) { Log = callback; }
    
    // 定位,计算当前位置
    void PredictCurrentPosition(Eigen::Vector3d &gyro_data, Eigen::Vector3d &acc_data, Eigen::Vector3d &mag_data,
                                Eigen::VectorXd &gps_data, Eigen::Vector3d &g_data, Eigen::Vector3d &ornt_data,
                                Eigen::Vector3d &road_data
    );

    // 获取当前GPS是否可用
    bool GetCurentGPSStatus();

    // 获取当前融合定位的输出
    routing::GNSSINS GetGNSSINS();

    // 设置采样频率
    void SetHz(double f);

    // 设置解压后的模型路径
    void SetModelPath(std::string model_path);
//    Eigen::VectorXd GPSJumpPointCompensate(routing::Status *status, Eigen::VectorXd &gps_bearing_queue);

};


#endif //LOCATION_LOCATION_H
