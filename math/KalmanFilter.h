//
// Created by yangcheng on 2018/11/25.
//

#ifndef LOCATION_KALMANFILTER_H
#define LOCATION_KALMANFILTER_H


#include <Eigen/Dense>


class KalmanFilter {

private:

    // t-1时刻状态
    Eigen::Vector4d x;
    // 转移矩阵
    Eigen::Matrix4d F;
    // 状态转移误差矩阵
    Eigen::Matrix4d Q;
    // 测量矩阵
    Eigen::Matrix4d H;
    // 测量误差矩阵)
    Eigen::Matrix4d R;
    // 协方差矩阵
    Eigen::Matrix4d P;

public:

    // 实例化
    KalmanFilter(Eigen::Vector4d &init_state);

    // 预测过程
    Eigen::Vector4d PredictState();

    Eigen::Matrix4d CalcPrioriCov();

    // 更新过程
    void UpdateState(Eigen::Vector4d &measure_state);

    // 由于每次GPS更新间隔并非常数,需手动更新矩阵F
    void SetF(double deltaT);

    // 过程随着运动不断更新deltas,即速度方差, P和Q
    void SetPQ(double varS);

    // 获取过滤后的
    Eigen::Vector4d GetState();

};


#endif //LOCATION_KALMANFILTER_H
