//
// Created by yangcheng on 2019/1/7.
//

#include "Eigen/Dense"

#ifndef LOCATION_OPTIMIZER_H
#define LOCATION_OPTIMIZER_H

using namespace Eigen;

// 优化器, 解决标定参数求解问题
class Optimizer {
public:

    void LevenbergMarquardt(MatrixXd &input_data, VectorXd *coef, double &gamma, double &epsilon, int &max_iter);

    void GaussNewton(MatrixXd &input_data, VectorXd *coef, double &epsilon, int &max_iter);

//private:

    MatrixXd EllipticalCaliJacobi(MatrixXd &input_data, VectorXd *coef);

    VectorXd EllipticalFx(MatrixXd &input_data, VectorXd *coef);
};


#endif //LOCATION_OPTIMIZER_H
