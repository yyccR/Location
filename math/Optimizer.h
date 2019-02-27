//
// Created by yangcheng on 2019/1/7.
//

#ifndef LOCATION_OPTIMIZER_H
#define LOCATION_OPTIMIZER_H

#include "Eigen/Dense"
#include "../system/Status.h"

// 优化器, 解决标定参数求解问题
class Optimizer {
public:

    void LevenbergMarquardt(Eigen::MatrixXd &input_data, double &R, Eigen::VectorXd *coef, double &gamma, double &epsilon, int &max_iter);

//    void LevenbergMarquardt(Eigen::MatrixXd &input_data, Parameters *parameters);

    void GaussNewton(Eigen::MatrixXd &input_data, double &R, Eigen::VectorXd *coef, double &epsilon, int &max_iter);

//    void GaussNewton(Eigen::MatrixXd &input_data, Parameters *parameters);

//private:

    Eigen::MatrixXd EllipticalCaliJacobi(Eigen::MatrixXd &input_data, Eigen::VectorXd *coef, double &R);

//    Eigen::MatrixXd EllipticalCaliJacobi(Eigen::MatrixXd &input_data, Parameters *parameters);

    Eigen::VectorXd EllipticalFx(Eigen::MatrixXd &input_data, Eigen::VectorXd *coef, double &R);

//    Eigen::VectorXd EllipticalFx(Eigen::MatrixXd &input_data, Parameters *parameters);
};


#endif //LOCATION_OPTIMIZER_H
