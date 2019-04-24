//
// Created by yangcheng on 2018/11/25.
//

#include "KalmanFilter.h"
#include "../sensor/GPS.h"

using namespace Eigen;


KalmanFilter::KalmanFilter(Eigen::Vector4d &init_state) {

    this->x = init_state;

    // 转移矩阵
    this->F << 1.0, 0.0, 1.0, 0.0,
            0.0, 1.0, 0.0, 1.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0;

    // 转移误差
    this->Q << 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.1, 0.0,
            0.0, 0.0, 0.0, 0.1;

    // 测量矩阵
    this->H << 1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0,0.0,1.0,0.0,
            0.0,0.0,0.0,1.0;

    // GPS噪声方差
    this->R << 0.00001, 0.0, 0.0, 0.0,
            0.0, 0.00001, 0.0, 0.0,
            0.0, 0.0, 0.1, 0.0001,
            0.0, 0.0, 0.0001, 0.1;


    // 协方差初始化
    this->P << 0.00001, 0.0, 0.0, 0.0,
            0.0, 0.00001, 0.0, 0.0,
            0.0, 0.0, 0.1, 0.0,
            0.0, 0.0, 0.0, 0.1;
}


Eigen::Vector4d KalmanFilter::PredictState() {

    GPS gps;
    double lng = this->x(0);
    double lat = this->x(1);
    double v_east = this->x(2);
    double v_north = this->x(3);
    double east_dist = v_east * this->F(0, 2);
    double north_dist = v_north * this->F(1, 3);
    double north_angle = 0.0;
    double east_angle = 90.0;
    double new_lat = gps.CalDestination(lng, lat, north_dist, north_angle)(1);
    double new_lng = gps.CalDestination(lng, lat, east_dist, east_angle)(0);

    Vector4d priori_State(new_lng, new_lat, v_east, v_north);
    return priori_State;
}

Eigen::Matrix4d KalmanFilter::CalcPrioriCov() {
    return this->F * this->P * this->F.transpose() + this->Q;
}

void KalmanFilter::UpdateState(Eigen::Vector4d &measure_state) {

    Vector4d prioriState = PredictState();
    Matrix4d prioriCov = CalcPrioriCov();
    // TODO: 求逆方法改进
    Matrix4d k = prioriCov * this->H.transpose() * (this->H * prioriCov * this->H.transpose() + this->R).inverse();
    Vector4d posteriorState = prioriState + k * (measure_state - this->H * prioriState);
    Matrix4d posteriorCov = (MatrixXd::Identity(4, 4) - k * this->H) * prioriCov;
    this->x = posteriorState;
    this->P = posteriorCov;
}


void KalmanFilter::SetF(double deltaT) {
    this->F(0,2) = deltaT;
    this->F(1,3) = deltaT;
}

void KalmanFilter::SetPQ(double varS) {
    this->P(2,2) = varS;
    this->P(3,3) = varS;

    this->Q(2,2) = varS;
    this->Q(3,3) = varS;
}

Eigen::Vector4d KalmanFilter::GetState() {
    return this->x;
}