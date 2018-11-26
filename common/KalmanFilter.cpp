//
// Created by yangcheng on 2018/11/25.
//

#include "KalmanFilter.h"

using namespace Eigen;
using namespace std;

//void KalmanFilter::KalmanFilter() {}
//
//void KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &state, MatrixXd &covMat, MatrixXd &tranMat, MatrixXd &measMat, MatrixXd &mCovMat, MatrixXd &eCovMat) {
    x = state;
    P = covMat;
    F = tranMat;
    H = measMat;
    R = mCovMat;
    Q = eCovMat;
    s = state.rows();
}

void KalmanFilter::Predict() {
    x_priori = F * x;
    p_priori = F * x * F.transpose() + Q;
}

void KalmanFilter::Correct(const VectorXd &measState) {
    MatrixXd k_gain = p_priori * H.transpose() * (H * p_priori * H.transpose() + R).inverse();
    x = x_priori + k_gain * (measState - H * x_priori);
    P = (MatrixXd::Identity(s,s) - k_gain * H) * p_priori;
}
