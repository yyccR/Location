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

    void LevenbergMarquardt(MatrixXd &input_data, VectorXd *coef);

    void GaussNewton(MatrixXd &input_data, VectorXd *coef);

private:

    MatrixXd EllipticalCaliJacobi(MatrixXd &input_data, VectorXd *coef);

};


#endif //LOCATION_OPTIMIZER_H
