//
// Created by yangcheng on 2019/1/7.
//
#include "iostream"
#include "Optimizer.h"

using namespace Eigen;

/**
 * LM(Levenberg-Marquardt)算法, 简化了牛顿法中Hessian矩阵的二阶项, 同时加入了阻尼因子
 *
 * @param input_data, n*3, n组数据,每组3维
 * @param coef 椭圆6参数，用于标定(加速计/地磁计)传感器
 * @param gamma, 初始化阻尼因子用, mu = gamma * max(A), A = Jacobi_t * Jacobi;
 * @param epsilon 迭代精度
 * @param max_iter 最高迭代次数
 */
void
Optimizer::LevenbergMarquardt(MatrixXd &input_data, double &R, VectorXd *coef, double &gamma, double &epsilon,
                              int &max_iter) {

    int data_nums = static_cast<int>(input_data.rows());
    // delta, coef的梯度
    VectorXd delta(6);
    // e_k = 1 - (x_k - coef(0))² * coef(3)² - (y_k - coef(1))² * coef(4)² - (z_k - coef(2))² * coef(5)²
    VectorXd e_k(data_nums);
    // Hessian矩阵, 忽略牛顿法中的二阶项
    MatrixXd hessian(6, 6);
    // 阻尼Hessian矩阵
    MatrixXd hessian_mu(6, 6);
    // g向量, Jacobi_t * e_k
    VectorXd g(6);

    // jacobi矩阵
    MatrixXd jacobi(data_nums, 6);

    // 初始化算法迭代参数
    int iter = 0;
    int v = 2;
    double mu = 0;
    bool found = false;

    // LM算法主流程
    // e_k 计算
    e_k = EllipticalFx(input_data, coef, R);
    // 椭球方程的Jacobi矩阵计算
    jacobi = EllipticalCaliJacobi(input_data, coef, R);
    // hssian矩阵计算
    hessian = jacobi.transpose() * jacobi;
    // 初始计算阻尼
    mu = gamma * hessian.maxCoeff();
    // 计算g
    g = jacobi.transpose() * e_k;


    while (!found && iter <= max_iter) {

        iter += 1;
        //std::cout << iter << std::endl;
        // hessin矩阵加入阻尼因子
        hessian_mu = hessian + mu * MatrixXd::Identity(6, 6);
        // 计算更新步长
        delta = hessian_mu.inverse() * (-1 * g);
        // 判断是否退出
        if (delta.norm() <= epsilon * ((*coef).norm() + epsilon)) {
            found = true;
        } else {

            // 计算可能的新参数
            VectorXd coef_new(6);
            coef_new(0) = (*coef)(0) + delta(0);
            coef_new(1) = (*coef)(1) + delta(1);
            coef_new(2) = (*coef)(2) + delta(2);
            coef_new(3) = (*coef)(3) + delta(3);
            coef_new(4) = (*coef)(4) + delta(4);
            coef_new(5) = (*coef)(5) + delta(5);

            // 计算rho, 为更新阻尼因子做准备
            double L0_Ldelta = 0.5 * delta.transpose() * (mu * delta - g);
            // 通用此处原算法只用到一组数据，修改为多组情况下取模
            double Fx_Fxdelta = e_k.norm() - EllipticalFx(input_data, &coef_new, R).norm();
//            double Fx_Fxdelta_v = Fx_Fxdelta.mean();
            // rho
//            double rho = Fx_Fxdelta_v / L0_Ldelta;
            double rho = Fx_Fxdelta / L0_Ldelta;
            // 根据rho大小更新mu
            if (rho > 0) {
                (*coef)(0) = coef_new(0);
                (*coef)(1) = coef_new(1);
                (*coef)(2) = coef_new(2);
                (*coef)(3) = coef_new(3);
                (*coef)(4) = coef_new(4);
                (*coef)(5) = coef_new(5);

                // 重新计算Jacobi, e_k, hessian, g
                e_k = EllipticalFx(input_data, coef, R);
                jacobi = EllipticalCaliJacobi(input_data, coef, R);
                hessian = jacobi.transpose() * jacobi;
                g = jacobi.transpose() * e_k;

                if (g.norm() < epsilon) {
                    found = true;
                }

                // 更新mu
                Vector2d temp(1 / 3.0, 1 - (2 * rho - 1) * (2 * rho - 1) * (2 * rho - 1));
                mu = mu * temp.maxCoeff();
                v = 2;
            } else {
                mu *= v;
                v *= 2;
            }
            std::cout << "delta.norm() " << delta.norm() << " g.norm() " << g.norm() << " iter " << iter << " found "
                      << found
                      << " rho " << rho << std::endl;
        }
    }
}

/**
 * 高斯牛顿法, 简化了牛顿法中Hessian矩阵的二阶项, 在某些情况下不收敛，待验证中。
 *
 * @param input_data, n*3, n组数据,每组3维
 * @param coef 椭圆6参数，用于标定(加速计/地磁计)传感器
 * @param epsilon 迭代精度
 * @param max_iter 最高迭代次数
 */
void Optimizer::GaussNewton(MatrixXd &input_data, double &R, VectorXd *coef, double &epsilon, int &max_iter) {

    int data_nums = static_cast<int>(input_data.rows());
    // delta, coef的梯度
    VectorXd delta(6);
    // e_k = 1 - (x_k - coef(0))² * coef(3)² - (y_k - coef(1))² * coef(4)² - (z_k - coef(2))² * coef(5)²
    VectorXd e_k(data_nums);
    // Hessian矩阵, 忽略牛顿法中的二阶项
    MatrixXd hessian(6, 6);
    // jacobi矩阵
    MatrixXd jacobi(data_nums, 6);

    double epsilon_temp = 10.0;
    int iter = 0;

    while (epsilon_temp > epsilon && iter <= max_iter) {
        // e_k 计算
        e_k = EllipticalFx(input_data, coef, R);

        // 椭球方程的Jacobi矩阵计算
        jacobi = EllipticalCaliJacobi(input_data, coef, R);
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

        epsilon_temp = delta.norm();//e_k.cwiseAbs().sum();//delta.sum();
        iter += 1;
//        std::cout << " iter " << iter << "\n jacobi \n" << jacobi  << "\n hessian \n" << hessian  << "\n hessian inverse \n" << hessian.inverse() << std::endl;
//        std::cout << "epsilon " << epsilon_temp << std::endl;
    }
}

/**
 * 椭圆方程的雅可比矩阵计算, 椭球公式如下：
 * e_k = R² - (x_k - coef(0))² * coef(3)² - (y_k - coef(1))² * coef(4)² - (z_k - coef(2))² * coef(5)²
 *
 * @param input_data n*3, n组数据,每组3维
 * @param coef 椭圆6参数，用于标定(加速计/地磁计)传感器
 * @return n*6 的雅可比矩阵
 */
MatrixXd Optimizer::EllipticalCaliJacobi(MatrixXd &input_data, VectorXd *coef, double &R) {

    int data_nums = static_cast<int>(input_data.rows());
    MatrixXd jacobiPileUp(data_nums, 6);

    for (int i = 0; i < data_nums; i++) {

        double ex = (input_data(i, 0) - (*coef)(0));
        double ey = (input_data(i, 1) - (*coef)(1));
        double ez = (input_data(i, 2) - (*coef)(2));
        double e = R*R - ex * ex * (*coef)(3) * (*coef)(3)
                   - ey * ey * (*coef)(4) * (*coef)(4)
                   - ez * ez * (*coef)(5) * (*coef)(5);

        // 对每个coef求导
        jacobiPileUp(i, 0) = 2*e* 2 * ex * (*coef)(3) * (*coef)(3);
        jacobiPileUp(i, 1) = 2*e* 2 * ey * (*coef)(4) * (*coef)(4);
        jacobiPileUp(i, 2) = 2*e* 2 * ez * (*coef)(5) * (*coef)(5);
        jacobiPileUp(i, 3) = 2*e* (-2) * (*coef)(3) * ex * ex;
        jacobiPileUp(i, 4) = 2*e* (-2) * (*coef)(4) * ey * ey;
        jacobiPileUp(i, 5) = 2*e* (-2) * (*coef)(5) * ez * ez;

    }

    return jacobiPileUp;
}

/**
 * 计算椭球方程误差, 椭球公式如下：
 * e_k = R² - (x_k - coef(0))² * coef(3)² - (y_k - coef(1))² * coef(4)² - (z_k - coef(2))² * coef(5)²
 *
 * @param input_data n*3, n组数据,每组3维
 * @param coef 椭圆6参数，用于标定(加速计/地磁计)传感器
 * @param R 椭圆半径
 * @return e_k 向量
 */
VectorXd Optimizer::EllipticalFx(MatrixXd &input_data, VectorXd *coef, double &R) {

    int data_nums = static_cast<int>(input_data.rows());
    // e_k = R² - (x_k - coef(0))² * coef(3)² - (y_k - coef(1))² * coef(4)² - (z_k - coef(2))² * coef(5)²
    VectorXd e_k(data_nums);

    for (int i = 0; i < data_nums; i++) {
        double ex = (input_data(i, 0) - (*coef)(0));
        double ey = (input_data(i, 1) - (*coef)(1));
        double ez = (input_data(i, 2) - (*coef)(2));

        double e = R*R - ex * ex * (*coef)(3) * (*coef)(3)
                   - ey * ey * (*coef)(4) * (*coef)(4)
                   - ez * ez * (*coef)(5) * (*coef)(5);
        e_k(i) = e * e;
    }

    return e_k;
}