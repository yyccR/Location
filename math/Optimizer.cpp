//
// Created by yangcheng on 2019/1/7.
//

#include "Optimizer.h"

/**
 * LM(Levenberg-Marquardt)算法, 简化了牛顿法中Hessian矩阵的二阶项, 同时加入了阻尼因子
 *
 * @param input_data, n*6, n组数据,每组6维
 * @param coef 椭圆6参数，用于标定(加速计/地磁计/陀螺仪)传感器。
 */
void Optimizer::LevenbergMarquardt(MatrixXd &input_data, VectorXd *coef) {

}

/**
 * 高斯牛顿法, 简化了牛顿法中Hessian矩阵的二阶项
 *
 * @param input_data, n*6, n组数据,每组6维
 * @param coef 椭圆6参数，用于标定(加速计/地磁计/陀螺仪)传感器。
 */
void Optimizer::GaussNewton(MatrixXd &input_data, VectorXd *coef) {

    int data_nums = static_cast<int>(input_data.rows());
    // delta, coef的梯度
    VectorXd delta(6);
    // e_k = 1 - (x_k - coef(0))² * coef(3)² - (y_k - coef(1))² * coef(4)² - (z_k - coef(2))² * coef(5)²
    VectorXd e_k(data_nums);
    // Hessian矩阵, 忽略牛顿法中的二阶项
    MatrixXd hessian(6, 6);
    // jacobi矩阵
    MatrixXd jacobi(data_nums, 6);

    // e_k 计算
    for (int i = 0; i < data_nums; i++) {

        double ex = (input_data(i, 0) - (*coef)(0));
        double ey = (input_data(i, 1) - (*coef)(1));
        double ez = (input_data(i, 2) - (*coef)(2));

        e_k(i) = 1 - ex * ex * (*coef)(3) * (*coef)(3)
                 - ey * ey * (*coef)(4) * (*coef)(4)
                 - ez * ez * (*coef)(5) * (*coef)(5);
    }

    // 椭球方程的Jacobi矩阵计算
    jacobi = EllipticalCaliJacobi(input_data, coef);
    // hssian矩阵计算
    hessian = jacobi.transpose() * jacobi;
    // 计算delta
    delta = hessian.inverse() * (jacobi.transpose() * e_k);

    // 更新coef
    (*coef)(0) -= delta(0);
    (*coef)(1) -= delta(1);
    (*coef)(2) -= delta(2);
    (*coef)(3) -= delta(3);
    (*coef)(4) -= delta(4);
    (*coef)(5) -= delta(5);
}

/**
 * 椭圆方程的雅可比矩阵计算, 公式如下：
 * e_k = 1 - (x_k - coef(0))² * coef(3)² - (y_k - coef(1))² * coef(4)² - (z_k - coef(2))² * coef(5)²
 *
 * @param input_data n*6, n组数据,每组6维
 * @param coef 椭圆6参数，用于标定(加速计/地磁计/陀螺仪)传感器
 * @return n*6 的雅可比矩阵
 */
MatrixXd Optimizer::EllipticalCaliJacobi(MatrixXd &input_data, VectorXd *coef) {

    int data_nums = static_cast<int>(input_data.rows());
    MatrixXd jacobiPileUp(data_nums, 6);

    for (int i = 0; i < data_nums; i++) {

        double ex = (input_data(i, 0) - (*coef)(0));
        double ey = (input_data(i, 1) - (*coef)(1));
        double ez = (input_data(i, 2) - (*coef)(2));

        // 对每个coef求导
        jacobiPileUp(i, 0) = 2 * ex * (*coef)(3) * (*coef)(3);
        jacobiPileUp(i, 1) = 2 * ey * (*coef)(4) * (*coef)(4);
        jacobiPileUp(i, 2) = 2 * ez * (*coef)(5) * (*coef)(5);
        jacobiPileUp(i, 3) = -2 * (*coef)(3) * ex * ex;
        jacobiPileUp(i, 4) = -2 * (*coef)(4) * ey * ey;
        jacobiPileUp(i, 5) = -2 * (*coef)(5) * ez * ez;

    }

    return jacobiPileUp;
}