//
// Created by yangcheng on 2019/1/14.
//

#include "Eigen/Dense"
#ifndef LOCATION_STATUS_H
#define LOCATION_STATUS_H



using namespace Eigen;

struct Position {
    // x,y,z轴的平面位置坐标
    double x;
    double y;
    double z;

    // x,y,z轴的球面位置坐标
    double lng;
    double lat;
    double altitude;
};

struct Velocity {
    // x,y,z轴的速度
    double v_x;
    double v_y;
    double v_z;
};

struct Attitude {
    // 姿态角, 弧度
    double roll;
    double pitch;
    double yaw;
};

struct Parameters{

    // 陀螺仪标定参数, v(offset_x,offset_y,offset_z)
    VectorXd gyro_coef;

    // 加速计标定参数, v(offset_x,offset_y,offset_z,scale_x,scale_y,scale_z)
    VectorXd acc_coef;

    // 地磁计标定参数, v(offset_x,offset_y,offset_z,scale_x,scale_y,scale_z)
    VectorXd mag_coef;

    // LM算法标定参数, 对加速计和地磁计通用
    // 初始化阻尼因子用, mu = gamma * max(A), A = Jacobi_t * Jacobi;
    double gamma;
    // 迭代精度
    double epsilon;
    // 最高迭代次数
    int max_step;

    // AHRS算法参数
    // 误差积分
    Vector3d err;
    // PID控制算法参数, 比例参数
    double ki;
    // PID控制算法参数, 积分参数
    double kp;
    // 采样频率的一半
    double halfT;

    // 当地重力加速度值
    double g;
};

class Status {
public:

    Position position;
    Velocity velocity;
    Attitude attitude;
    Parameters parameters;

    Position GetPosition() const;

    Velocity GetVelocity() const;

    Attitude GetAttitude() const;

    Parameters GetParameters() const;

};

#endif //LOCATION_STATUS_H