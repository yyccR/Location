//
// Created by yangcheng on 2018/11/25.
//

#ifndef LOCATION_KALMANFILTER_H
#define LOCATION_KALMANFILTER_H


#include <Eigen/Dense>
#include <vector>
#include <iostream>


class KalmanFilter {

public:
    // state vecotr
    Eigen::VectorXd x;
    // priori state vector
    Eigen::VectorXd x_priori;
    // covariance matrix
    Eigen::MatrixXd P;
    // priori covariance matrix
    Eigen::MatrixXd p_priori;
    // transistion matrix
    Eigen::MatrixXd F;
    // measurement matrix
    Eigen::MatrixXd H;
    // measurement covariance matrix
    Eigen::MatrixXd R;
    // error covariance matrix
    Eigen::MatrixXd Q;
    // input size;
    Eigen::Index s;

    /**
    * Constructor
    */
    KalmanFilter();

    /**
    * Destructor
    */
    virtual ~KalmanFilter();

    void Init(Eigen::VectorXd &state, Eigen::MatrixXd &covMat, Eigen::MatrixXd &tranMat, Eigen::MatrixXd &measMat,
              Eigen::MatrixXd &mCovMat, Eigen::MatrixXd &eCovMat);

    void Correct(const Eigen::VectorXd &measState);

    void Predict();
};


#endif //LOCATION_KALMANFILTER_H
