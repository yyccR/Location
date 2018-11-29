//
// Created by yangcheng on 2018/11/25.
//

#ifndef LOCATION_KALMANFILTER_H
#define LOCATION_KALMANFILTER_H


#include <Eigen\Dense>
#include <vector>
#include <iostream>

using namespace std;
using namespace Eigen;

class KalmanFilter {

public:
    // state vecotr
    VectorXd x;
    // priori state vector
    VectorXd x_priori;
    // covariance matrix
    MatrixXd P;
    // priori covariance matrix
    MatrixXd p_priori;
    // transistion matrix
    MatrixXd F;
    // measurement matrix
    MatrixXd H;
    // measurement covariance matrix
    MatrixXd R;
    // error covariance matrix
    MatrixXd Q;
    // input size;
    Index s;

    /**
    * Constructor
    */
    KalmanFilter();

    /**
    * Destructor
    */
    virtual ~KalmanFilter();

    void Init(VectorXd &state, MatrixXd &covMat, MatrixXd &tranMat, MatrixXd &measMat, MatrixXd &mCovMat, MatrixXd &eCovMat);

    void Correct(const VectorXd &measState);

    void Predict();
};


#endif //LOCATION_KALMANFILTER_H
