//
// Created by yangcheng on 2019/1/14.
//

#ifndef LOCATION_STATUS_H
#define LOCATION_STATUS_H

#include "../include/eigen3/Eigen/Dense"

namespace routing {

    // 用于最终输出
    struct GNSSINS {
        double lng;
        double lat;
        double altitude;
        double accuracy;
        double speed;
        double bearing;
    };

    struct Position {
        // x,y,z轴的平面位置坐标
        double x;
        double y;
        double z;

        // 经纬度,海拔
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

        // 姿态四元数
        Eigen::Vector4d q_attitude;
    };

    struct Parameters{

        // 弱GPS精度阈值
        double weak_gps;

        // 陀螺仪标定参数, v(offset_x,offset_y,offset_z)
        Eigen::VectorXd gyro_coef;

        // 加速计标定参数, v(offset_x,offset_y,offset_z,scale_x,scale_y,scale_z)
        Eigen::VectorXd acc_coef;

        // 地磁计标定参数, v(offset_x,offset_y,offset_z,scale_x,scale_y,scale_z)
        Eigen::VectorXd mag_coef;

        // LM算法标定参数, 对加速计和地磁计通用
        // 初始化阻尼因子用, mu = gamma * max(A), A = Jacobi_t * Jacobi;
        double gamma;
        // 迭代精度
        double epsilon;
        // 最高迭代次数
        int max_step;

        // AHRS算法参数
        // 误差积分
        Eigen::Vector3d err;
        // PID控制算法参数, 比例参数
        double ki;
        // PID控制算法参数, 积分参数
        double kp;
        // 采样频率的一半
        double halfT;
        // 采样时间间隔
        double t;
        // 等间隔时间内GPS的运动距离阈值,用于判断高精度但是漂移的点
        double move_distance_threshod;
        // 利用惯导计算位置的次数;
        int ins_count;
        // GPS初始状态计数
        int gps_count;
        // 导航GPS初始化后，记录多久后接入ins
        int gps_init_threshold;
        // gps上一个点,经纬度,时间戳
        double gps_pre_lng;
        double gps_pre_lat;
        double gps_pre_t;

        // 当地重力加速度值
        double g;
        // 当地场强模值
        double mag;
        // 地球自转角速度
        double we;
        // 地球半径
        double R;
    };

    class Status {
    public:

        GNSSINS gnssins;
        Position position;
        Velocity velocity;
        Attitude attitude;
        Parameters parameters;

        Position GetPosition() const;

        Velocity GetVelocity() const;

        Attitude GetAttitude() const;

        Parameters GetParameters() const;

        void Init();

    };

}

#endif //LOCATION_STATUS_H
