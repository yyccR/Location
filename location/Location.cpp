//
// Created by yangcheng on 2019/1/14.
//

#include <cmath>
#include "../sensor/GPS.h"
#include "../sensor/Accelerometer.h"
#include "../sensor/Gravity.h"
#include "../sensor/Compass.h"
#include "../models/AHRS.h"
#include "../models/StrapdownAHRS.h"
#include "Location.h"
#include "../math/LPF.h"
#include "iostream"

using namespace Eigen;
using namespace routing;

/**
 * Location 初始化。
 */
Location::Location() {
    // 初始化参数
    this->status.Init();
    LPF lpf;
    lpf.LowPassFilter2ndFactorCal(&status);
}

Location::~Location() {}

/**
 * 定位,计算当前位置
 *
 * @param gyro_data, 陀螺仪原始数据, w(x,y,z)
 * @param acc_data, 加速计原始数据, a(x,y,z)
 * @param mag_data, 地磁计原始数据, m(x,y,z)
 * @param gps_data, GPS原始数据, gps(lng,lat,alt,accuracy,speed,bearing,t)
 * @param g_data, 重力感应数据, g(x,y,z)
 * @param ornt_data, 方向传感器数据, o(roll,pitch,yaw)
 * @param road_data, 道路方向数据,包含距离下个路口距离和当前位置道路方向, 道路类型编码, v(distance, bearing, code)
 * @param status, 状态容器, 包含位置,姿态,速度,参数等信息
 */
void Location::PredictCurrentPosition(Vector3d &gyro_data, Vector3d &acc_data, Vector3d &mag_data, VectorXd &gps_data,
                                      Vector3d &g_data, Vector3d &ornt_data, Vector3d &road_data) {

    // 记录起始位置和当前位置
    double start_x = status.position.x;
    double start_y = status.position.y;
    double start_lng = status.position.lng;
    double start_lat = status.position.lat;

    // 更新道路状态
    UpdateRoadType(&status, road_data);
    // 更新惯性位置,速度
    Accelerometer accelerometer;
//    accelerometer.PositionIntegral(&status, final_acc_lpf, status.parameters.t);
    Quaternions quaternions;
    // 方向数据修正
    LPF lpf;
    Vector3d ornt_filter = lpf.LowPassFilter4Ornt(&status, ornt_data);
    Vector4d attitude = quaternions.GetQFromEuler(ornt_filter);
    accelerometer.StrapdownUpdateVelocityPosition(&status, acc_data, attitude, g_data);

    // 指南针波动情况
    Compass compass;
    bool is_compass_vaild = compass.IsCompassVaild(&status, ornt_data);

    // 判断手机是否摆放发生变化
    Gravity gravity;
    bool is_shaking = gravity.IsShaking(&status, g_data);

    // 判断是否以及行走到路口一定范围内
    bool is_near_cross = status.parameters.dist_from_pre_cross < status.parameters.min_dist_to_cross ||
                         status.parameters.dist_to_next_cross < status.parameters.min_dist_to_cross;

//    std::cout << "is_near_cross " << is_near_cross << " pre dist " << status.parameters.dist_from_pre_cross
//              << " next dist " << status.parameters.dist_to_next_cross << std::endl;

    // 获取GPS精度
    GPS gps;
    // 计算传感器运动距离
    double end_x = status.position.x;
    double end_y = status.position.y;
    double distance = sqrt((end_x - start_x) * (end_x - start_x) + (end_y - start_y) * (end_y - start_y));

    // 判断ins是否走的距离达到限制最大距离
    bool is_ins_move_not_too_far = status.parameters.ins_dist < status.parameters.max_ins_dist;
    // 判断是否采用GPS数据
    bool is_gps_valid = gps.IsGPSValid(&status, &gps_data);
    if (!is_gps_valid) {
        // 采用惯导更新经纬度
        if (is_shaking || !is_compass_vaild || !is_near_cross) {
            // 更新道路方向和方向传感器Z轴方向, 当GPS精度低或不可用一定时间后
            UpdateZaxisWithGPSAndRoad(&status, gps_data, ornt_filter, road_data);
        }
        // 获取航向角
        double heading_no_limit = ornt_filter(2) + status.parameters.diff_gps_ornt;
        double heading;
        if (heading_no_limit > 360.0) {
            heading = heading_no_limit - 360.0;
        } else if(heading_no_limit < 0){
            heading = heading_no_limit + 360.0;
        } else {
            heading = heading_no_limit;
        }
        status.attitude.yaw = heading;
        if (is_ins_move_not_too_far) {
            // 计算航向角
            Vector2d gps_new = gps.CalDestination(start_lng, start_lat, distance, heading);
            // 更新经纬度
            status.position.lng = gps_new(0);
            status.position.lat = gps_new(1);
            // 更新相关参数
            status.parameters.ins_count += 1;
            status.parameters.ins_dist += distance;
        }
//        std::cout << status.parameters.gps_pre_bearing << " " << ornt_filter(2) << " " << status.parameters.diff_gps_ornt << " "
//                  << heading_no_limit << " " << road_data(1) << std::endl;
    } else {
        // 采用GPS数据更新经纬度和方位角
        double gps_speed = gps_data(4);
        double gps_bearing = gps_data(5);
        status.position.lng = gps_data(0);
        status.position.lat = gps_data(1);
        status.position.altitude = gps_data(2);
        status.attitude.yaw = gps_bearing;
        gps.UpdateVelocity(&status, gps_speed, gps_bearing);
        // 更新相关参数值,gps(lng,lat,alt,accuracy,speed,bearing,t)
        status.parameters.gps_pre_lng = gps_data(0);
        status.parameters.gps_pre_lat = gps_data(1);
        status.parameters.gps_pre_altitude = gps_data(2);
        status.parameters.gps_pre_accuracy = gps_data(3);
        status.parameters.gps_pre_speed = gps_speed;
        status.parameters.gps_pre_bearing = gps_bearing;
        status.parameters.gps_pre_t = gps_data(6);
        // 时间t影响因子自调整
        AutoAdjustTFactor(&status, gps_data, status.parameters.ins_dist);
        // 更新GPS方向和方向传感器Z轴方向, 当GPS可用且精度高时
        UpdateZaxisWithGPSAndRoad(&status, gps_data, ornt_filter, road_data);
        // 更新其他INS变量
        status.parameters.gps_count += 1;
        status.parameters.ins_count = 0;
        status.parameters.ins_dist = 0;
        // 更新融合定位结果输出
        status.gnssins.accuracy = gps_data(3);
        status.gnssins.speed = gps_speed;
        // 每个x,y,z都是相对与上一个准确的GPS数据。
        status.position.x = 0.0;
        status.position.y = 0.0;
        status.position.z = 0.0;
//        std::cout << gps_data(0) << " " << gps_data(1) << std::endl;
    }
//    std::cout << status.parameters.gps_pre_bearing << " " << ornt_data(2) << " " << ornt_filter(2) << " "
//              << status.parameters.diff_gps_ornt << " " << status.attitude.yaw << " " << road_data(1) << " "
//              << status.parameters.ins_count << " " << gps_data(0) << " " << gps_data(1) << std::endl;

    // 更新融合定位的结果，精度沿用GPS信号好时的精度,速度由于加速计计算的是三个方位的速度，故速度还是沿用GPS的速度
    status.gnssins.lng = status.position.lng;
    status.gnssins.lat = status.position.lat;
    status.gnssins.altitude = status.position.altitude;
    status.gnssins.bearing = status.attitude.yaw;

}

void Location::SetHz(double f) {
    this->status.parameters.Hz = f;
    this->status.parameters.acc_hz = f / 2.0;
    this->status.parameters.halfT = 1.0 / (f * 2.0);
    this->status.parameters.t = 1.0 / (f * (this->status.parameters.static_t_factor));
}


/**
 * 获取当前融合定位结果作为输出
 * @return
 */
GNSSINS Location::GetGNSSINS() {
    return this->status.gnssins;
}


/**
 * 获取当前位置
 * @return
 */
Position Location::GetCurrentPosition() {
    return this->status.position;
}

/**
 * 获取当前方位角
 * @return
 */
double Location::GetCurrentBearing() {
    return this->status.attitude.yaw;
}

/**
 * 自调节机制, 利用GPS运动距离调整t的放大因子
 *
 * @param status
 * @param gps_data, gps(lng,lat,alt,accuracy,speed,bearing,t)
 * @param ins_distance
 */
void Location::AutoAdjustTFactor(routing::Status *status, Eigen::VectorXd &gps_data, double ins_distance) {

    static int cnt = 0;
    static MatrixXd gps_queue(3, 7);
    static Vector3d ins_move_dist(3);

    if (cnt < 3) {
        gps_queue.row(cnt) = gps_data;
        ins_move_dist(cnt) = ins_distance;
        cnt += 1;
    } else {
        GPS gps;
        double lng1 = gps_queue(0, 0);
        double lat1 = gps_queue(0, 1);
        double lng2 = gps_queue(1, 0);
        double lat2 = gps_queue(1, 1);
        double lng3 = gps_queue(2, 0);
        double lat3 = gps_queue(2, 1);
        double gps_dist = gps.CalDistance(lng1, lat1, lng2, lat2) + gps.CalDistance(lng2, lat2, lng3, lat3);
        double ins_dist = ins_move_dist(1) + ins_move_dist(2);
        double deltaT = (gps_queue(2, 6) - gps_queue(0, 6)) / 1000.0;
        if (gps_dist != 0.0 && ins_dist != 0.0 && deltaT != 0.0) {
            (*status).parameters.move_t_factor *= sqrt(gps_dist / ins_dist);
        }
        // 先进先出
        gps_queue.row(0) = gps_queue.row(1);
        gps_queue.row(1) = gps_queue.row(2);
        gps_queue.row(2) = gps_data;
        ins_move_dist(0) = ins_move_dist(1);
        ins_move_dist(1) = ins_move_dist(2);
        ins_move_dist(2) = ins_distance;
    }
}

/**
 * 方向传感器和GPS,道路方向差值修正
 *
 * @param status
 * @param gps_data, gps(lng,lat,alt,accuracy,speed,bearing,t)
 * @param ornt_data, v(x,y,z)
 * @param road_data, 道路方向数据,包含距离下个路口距离,和当前点的瞬时方向,以及当前道路类型编码, v(distance, heading, code)
 */
void Location::UpdateZaxisWithGPSAndRoad(routing::Status *status, Eigen::VectorXd &gps_data, Eigen::Vector3d &ornt_data,
                                         Eigen::Vector3d &road_data) {
    static MatrixXd gps_queue((*status).parameters.queue_gps_ornt, 7);
    static MatrixXd ornt_queue((*status).parameters.queue_gps_ornt, 3);
    static MatrixXd road_queue((*status).parameters.queue_gps_ornt, 3);
    static int gps_ornt_cnt = 0;
    static int road_ornt_cnt = 0;

    if (gps_ornt_cnt < (*status).parameters.queue_gps_ornt || road_ornt_cnt < (*status).parameters.queue_gps_ornt) {
//    if (gps_ornt_cnt < (*status).parameters.queue_gps_ornt) {
        // gps方向队列
        if (gps_ornt_cnt < (*status).parameters.queue_gps_ornt &&
            gps_data(4) > (*status).parameters.gps_static_speed_threshold) {
            gps_queue.row(gps_ornt_cnt) = gps_data;
            ornt_queue.row(gps_ornt_cnt) = ornt_data;
            gps_ornt_cnt += 1;
        }
        // 道路方向队列
        if (road_ornt_cnt < (*status).parameters.queue_gps_ornt && road_data(0) != 0.0 && road_data(1) != 0.0) {
            road_queue.row(road_ornt_cnt) = road_data;
            road_ornt_cnt += 1;
        }
    } else {
        if (gps_data(0) == 0.0 && gps_data(1) == 0.0) {
            if ((*status).parameters.ins_count >
                (*status).parameters.Hz * (*status).parameters.least_gap_time_for_using_road) {
                // 没gps信号, 且超过一定时间后, 采用道路方向做修正
                VectorXd road_bearing = road_queue.col(1);
                VectorXd ornt_bearing = ornt_queue.col(2);
                double diff_road_ornt = (road_bearing - ornt_bearing).mean();
                // TODO: 当距下个路口一定距离同时方向差别在一定范围内,采用道路方向修正指南针方向
                (*status).parameters.diff_gps_ornt = diff_road_ornt;
//                if (road_data(0) != 0.0 && road_data(1) != 0.0) {
//                    for (int i = 0; i < (*status).parameters.queue_gps_ornt - 1; i++) {
//                        road_queue.row(i) = road_queue.row(i + 1);
//                        ornt_queue.row(i) = ornt_queue.row(i + 1);
//                    }
//                    road_queue.row((*status).parameters.queue_gps_ornt - 1) = road_data;
//                    ornt_queue.row((*status).parameters.queue_gps_ornt - 1) = ornt_data;
//                }
            } else {
                // 没信号,但是没有超过一定时间间隔, 仍用当前指南针和上个GPS点修正
                VectorXd gps_bearing = gps_queue.col(5);
                VectorXd ornt_bearing = ornt_queue.col(2);
                double diff_gps_ornt = (gps_bearing - ornt_bearing).mean();
                (*status).parameters.diff_gps_ornt = diff_gps_ornt;
//                for (int i = 0; i < (*status).parameters.queue_gps_ornt - 1; i++) {
//                    ornt_queue.row(i) = ornt_queue.row(i + 1);
//                }
//                ornt_queue.row((*status).parameters.queue_gps_ornt - 1) = ornt_data;
//                // 不用道路信息的时候也要更新道路状态
//                if (road_data(0) != 0.0 && road_data(1) != 0.0) {
//                    for (int i = 0; i < (*status).parameters.queue_gps_ornt - 1; i++) {
//                        road_queue.row(i) = road_queue.row(i + 1);
//                    }
//                    road_queue.row((*status).parameters.queue_gps_ornt - 1) = road_data;
//                }
            }
        } else {
            // 有gps信号时尽量采用gps信号做辅助修正
            VectorXd gps_bearing = gps_queue.col(5);
            VectorXd ornt_bearing = ornt_queue.col(2);
            double diff_gps_ornt = (gps_bearing - ornt_bearing).mean();
            (*status).parameters.diff_gps_ornt = diff_gps_ornt;
//            if (gps_data(4) > (*status).parameters.gps_static_speed_threshold) {
//                for (int i = 0; i < (*status).parameters.queue_gps_ornt - 1; i++) {
//                    gps_queue.row(i) = gps_queue.row(i + 1);
//                    ornt_queue.row(i) = ornt_queue.row(i + 1);
//                }
//                gps_queue.row((*status).parameters.queue_gps_ornt - 1) = gps_data;
//                ornt_queue.row((*status).parameters.queue_gps_ornt - 1) = ornt_data;
//            }
//            // 不用道路信息的时候也要更新道路状态
//            if (road_data(0) != 0.0 && road_data(1) != 0.0) {
//                for (int i = 0; i < (*status).parameters.queue_gps_ornt - 1; i++) {
//                    road_queue.row(i) = road_queue.row(i + 1);
//                }
//                road_queue.row((*status).parameters.queue_gps_ornt - 1) = road_data;
//            }
        }

        // 更新GPS数据
        if (gps_data(0) != 0.0 && gps_data(1) != 0.0 && gps_data(4) > (*status).parameters.gps_static_speed_threshold) {
            for (int i = 0; i < (*status).parameters.queue_gps_ornt - 1; i++) {
                gps_queue.row(i) = gps_queue.row(i + 1);
            }
            gps_queue.row((*status).parameters.queue_gps_ornt - 1) = gps_data;
        }

        // 更新道路数据
        if (road_data(0) != 0.0 && road_data(1) != 0.0) {
            for (int i = 0; i < (*status).parameters.queue_gps_ornt - 1; i++) {
                road_queue.row(i) = road_queue.row(i + 1);
            }
            road_queue.row((*status).parameters.queue_gps_ornt - 1) = road_data;
        }

        // 更新指南针数据
        for (int i = 0; i < (*status).parameters.queue_gps_ornt - 1; i++) {
            ornt_queue.row(i) = ornt_queue.row(i + 1);
        }
        ornt_queue.row((*status).parameters.queue_gps_ornt - 1) = ornt_data;

    }
//    std::cout << gps_queue(0,5) << " " << road_queue(0,1) << " " << ornt_queue(0,2) << " "
//              <<  (*status).parameters.diff_gps_ornt << std::endl;
}

/**
 * 更新道路状态
 * @param status
 * @param road_data, v(distance, bearing, code)
 */
void Location::UpdateRoadType(routing::Status *status, Eigen::Vector3d &road_data) {
    static double jump_point = 0.0;

    if (road_data(0) != 0.0 && road_data(1) != 0.0) {
        (*status).parameters.road_type = road_data(2);

        if (road_data(0) > (*status).parameters.dist_to_next_cross) {
            jump_point = road_data(0);
            (*status).parameters.dist_from_pre_cross = 0.0;
        } else {
            if (jump_point > road_data(0)) {
                (*status).parameters.dist_from_pre_cross = jump_point - road_data(0);
            }
        }
        (*status).parameters.dist_to_next_cross = road_data(0);
    } else {
        (*status).parameters.road_type = 0.0;
        (*status).parameters.dist_to_next_cross = 100000.0;
        (*status).parameters.dist_from_pre_cross = 100000.0;
    }

}


//Eigen::VectorXd Location::GPSJumpPointCompensate(routing::Status *status, Eigen::VectorXd &gps_bearing_queue) {
//
//
//}