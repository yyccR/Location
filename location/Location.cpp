//
// Created by yangcheng on 2019/1/14.
//

#include <cmath>
#include "Location.h"
#include <iostream>
#include <fstream>
#include "../sensor/GPS.h"
#include "../sensor/Accelerometer.h"
#include "../sensor/Gravity.h"
#include "../sensor/Compass.h"
#include "../models/AHRS.h"
#include "../models/StrapdownAHRS.h"
#include "../math/LPF.h"
#include "../models/XgboostDetector.h"

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
//    LoadStopDetectModel();
}

Location::~Location() {

}

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

    // 行车状态预测
//    PredictStopStatus(gyro_data, acc_data, mag_data, g_data, ornt_data);
    // 更新道路状态
    UpdateRoadType(road_data);
    // 更新惯性位置,速度
    Accelerometer accelerometer;
//    accelerometer.PositionIntegral(&status, final_acc_lpf, status.parameters.t);
    Quaternions quaternions;
    // 方向数据修正
    LPF lpf;
    Vector3d ornt_filter = lpf.LowPassFilter4Ornt(&status, ornt_data);
    Vector4d attitude = quaternions.GetQFromEuler(ornt_filter);
    AutoAdjustMovingFactor();
    accelerometer.StrapdownUpdateVelocityPosition(&status, acc_data, attitude, g_data);

    // 指南针波动情况
    Compass compass;
    bool is_compass_vaild = compass.IsCompassVaild(&status, ornt_data);

    // 判断手机是否摆放发生变化
    Gravity gravity;
    bool is_shaking = gravity.IsShaking(&status, g_data);

    // 判断是否以及行走到路口一定范围内
    bool is_near_cross = status.parameters.dist_from_pre_cross < status.parameters.min_dist_from_pre_cross ||
                         status.parameters.dist_to_next_cross < status.parameters.min_dist_to_next_cross;

    // 判断是否处于重新规划线路的时间段内
    bool is_routing = IsRouting(ornt_filter, road_data);

    // 判断指南针跟道路的方向变化是否一直
//    bool is_same_change = IsRoadCompassSameRange(&status, ornt_filter, road_data);

    // 判断是否偏航
//    bool is_off_course = IsOffCourse(&status, ornt_filter, road_data);

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
    status.parameters.is_current_gps_valid = is_gps_valid;
    if (!is_gps_valid) {

        // 采用惯导更新经纬度
        double heading_no_limit;
//        if ((is_shaking || !is_compass_vaild) || (!is_near_cross || is_same_change)) {
        if ((is_shaking || !is_compass_vaild) || (!is_near_cross && !is_routing)) {
//        if ((is_shaking || !is_compass_vaild) || !is_near_cross) {
            // 更新道路方向和方向传感器Z轴方向, 当GPS精度低或不可用一定时间后
            UpdateZaxisWithRoad(ornt_filter, road_data);
            heading_no_limit = ornt_filter(2) + status.parameters.diff_road_ornt;
        } else {
            if (status.parameters.ins_count >
                status.parameters.Hz * status.parameters.least_gap_time_for_using_road) {
                // 当无信号运动超过一定时间,此时用道路差值修正,GPS方向已经比较旧
                heading_no_limit = ornt_filter(2) + status.parameters.diff_road_ornt;
            } else {
                // 当无信号运动还没超过一定时间,此时用GPS方向
                heading_no_limit = ornt_filter(2) + status.parameters.diff_gps_ornt;
            }
        }

        // 限制取值范围
        double heading;
        if (heading_no_limit > 360.0) {
            heading = heading_no_limit - 360.0;
        } else if (heading_no_limit < 0) {
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
//        AutoAdjustTFactor(&status, gps_data, status.parameters.ins_dist);
        // 更新GPS方向和方向传感器Z轴方向, 当GPS可用且精度高时
        UpdateZaxisWithGPS(gps_data, ornt_filter);
        // 更新其他INS变量
        status.parameters.gps_count += 1;
        status.parameters.ins_count = 0;
        status.parameters.ins_dist = 0;
        status.parameters.move_t_factor = 1.0;
        // 更新融合定位结果输出
        status.gnssins.accuracy = gps_data(3);
        status.gnssins.speed = gps_speed;
        // 每个x,y,z都是相对与上一个准确的GPS数据。
        status.position.x = 0.0;
        status.position.y = 0.0;
        status.position.z = 0.0;
    }
//
//    std::string log_msg = std::to_string(status.parameters.gps_pre_bearing) + " " + std::to_string(ornt_filter(2)) + " "
//                          + std::to_string(status.parameters.diff_gps_ornt) + " "
//                          + std::to_string(status.parameters.diff_road_ornt) + " "
//                          + std::to_string(status.attitude.yaw) + " " + std::to_string(road_data(1)) + " "
//                          + std::to_string(road_data(0)) + " " + std::to_string(status.parameters.ins_count) + " "
//                          + std::to_string(gps_data(0)) + " " + std::to_string(gps_data(1)) + " "
//                          + std::to_string(((is_shaking || !is_compass_vaild) || (!is_near_cross)))
//                          + " " + std::to_string(is_shaking) + " " + std::to_string(is_compass_vaild) + " "
//                          + std::to_string(!is_near_cross) + " " + std::to_string(status.parameters.dist_to_next_cross)
//                          + " " + std::to_string(status.parameters.dist_from_pre_cross);
//    Log(log_msg);
//    std::cout << log_msg << std::endl;
//    std::cout << status.parameters.gps_pre_bearing << " " << ornt_filter(2) << " "
//              << status.parameters.diff_gps_ornt << " " << status.parameters.diff_road_ornt
//              << " " << status.attitude.yaw << " " << road_data(1) <<
//              " " << road_data(0) << " "
//              << status.parameters.ins_count << " " << gps_data(0) << " " << gps_data(1)
//              << " " << ((is_shaking || !is_compass_vaild) || (!is_near_cross || is_same_change)) << " "
//              << is_shaking << " " << is_same_change << " " << !is_near_cross
//              << " " << status.parameters.dist_to_next_cross << " "
//              << status.parameters.dist_from_pre_cross << std::endl;

    // 约定当惯导起作用时,精度返回99.99
    if (status.parameters.ins_count >
        status.parameters.Hz * status.parameters.least_gap_time_for_using_road)
        status.gnssins.accuracy = 99.99;
    // 更新融合定位的结果，精度沿用GPS信号好时的精度,速度由于加速计计算的是三个方位的速度，故速度还是沿用GPS的速度
    status.gnssins.lng = status.position.lng;
    status.gnssins.lat = status.position.lat;
    status.gnssins.altitude = status.position.altitude;
    status.gnssins.bearing = status.attitude.yaw;


}

/**
 * 设置传感器频率
 * @param f
 */
void Location::SetHz(double f) {
    this->status.parameters.Hz = f;
    this->status.parameters.acc_hz = f / 2.0;
    this->status.parameters.halfT = 1.0 / (f * 2.0);
    this->status.parameters.t = 1.0 / (f * (this->status.parameters.static_t_factor));
}

/**
 * 设置模型路径
 * @param model_path
 */
void Location::SetModelPath(std::string model_path) {
    this->status.parameters.stop_detector_model_path = model_path;
}

/**
 * 判断文件是否有效
 * @param model_path
 * @return
 */
bool Location::IsFileVaild(std::string &model_path) {
    std::ifstream inFile;
    inFile.open(model_path);
    if (inFile) {
        inFile.close();
        return true;
    } else {
        inFile.close();
        return false;
    }
}

/**
 * 加载模型
 */
void Location::LoadStopDetectModel() {
    std::string model_path = this->status.parameters.stop_detector_model_path;
    std::string model_name = "stopDetector.model";
    if (model_path.find(model_name) != std::string::npos) {
        bool is_file_vaild = IsFileVaild(model_path);
        if (is_file_vaild) {
            this->stopDetection = std::make_shared<XgboostDetector>(model_path);
        }
    } else {
        std::string model_full_path = model_path + "/stopDetector.model";
        bool is_file_vaild = IsFileVaild(model_full_path);
        if (is_file_vaild) {
            this->stopDetection = std::make_shared<XgboostDetector>(model_path);
        }
    }
}

/**
 * 预测行车状态
 *
 * @param gyro_data
 * @param acc_data
 * @param mag_data
 * @param g_data
 * @param ornt_data
 */
void Location::PredictStopStatus(Eigen::Vector3d &gyro_data, Eigen::Vector3d &acc_data, Eigen::Vector3d &mag_data,
                                 Eigen::Vector3d &g_data, Eigen::Vector3d &ornt_data) {

    static Vector3d pre_ornt_data(-999.0, -999.0, -999.0);
    static Vector3d pre_mag_data(-999.0, -999.0, -999.0);
    int total_detect_size = (int) (this->status.parameters.stop_status_window * this->status.parameters.Hz);
    static int cnt = 0;
    static VectorXd stop_status_v(total_detect_size);

    Quaternions quaternions;
    Accelerometer accelerometer;

    if (pre_ornt_data(0) != -999.0 && pre_ornt_data(1) != -999.0 && pre_ornt_data(2) != -999.0) {

        Vector3d ornt_diff = ornt_data - pre_ornt_data;
        Vector4d q = quaternions.GetQFromEuler(ornt_data);
        Matrix3d dcm = quaternions.GetDCMFromQ(q);

        Vector3d acc_v = acc_data;
        Vector3d acc_n = dcm * acc_v;
        Vector3d acc_v_norm = accelerometer.Normalise(acc_v);
        Vector3d acc_n_norm = accelerometer.Normalise(acc_n);

        Vector3d g_v = g_data;
        Vector3d g_n = dcm * g_v;
        Vector3d g_v_norm = accelerometer.Normalise(g_v);
        Vector3d g_n_norm = accelerometer.Normalise(g_n);
        Vector3d a_diff = acc_n - g_n;

        Vector3d gyro_v = gyro_data;
        Vector3d gyro_v_norm = accelerometer.Normalise(gyro_v);

        Vector3d mag_v_diff = (dcm * mag_data) - (dcm * pre_mag_data);
        Vector3d mag_v_norm = accelerometer.Normalise(mag_v_diff);

        VectorXd input_data(27);
        input_data << acc_v_norm(0), acc_v_norm(1), acc_v_norm(2), acc_n_norm(0), acc_n_norm(1), acc_n_norm(2),
                g_v_norm(0), g_v_norm(1), g_v_norm(2), g_n_norm(0), g_n_norm(1), g_n_norm(2),
                gyro_v_norm(0), gyro_v_norm(1), gyro_v_norm(2), mag_v_norm(0), mag_v_norm(1), mag_v_norm(2),
                mag_v_diff(0), mag_v_diff(1), mag_v_diff(2), a_diff(0), a_diff(1), a_diff(2),
                ornt_diff(0), ornt_diff(1), ornt_diff(2);

        if (this->stopDetection) {
            bool detect_res = this->stopDetection->IsStopping(input_data);
            if (cnt < total_detect_size) {
                stop_status_v(cnt) = detect_res;
                cnt += 1;
            } else {
                // 队列先进先出
                for (int i = 0; i < total_detect_size - 1; ++i) {
                    stop_status_v(i) = stop_status_v(i + 1);
                    stop_status_v(i) = stop_status_v(i + 1);
                }
                stop_status_v(total_detect_size - 1) = detect_res;
                if (stop_status_v.sum() / total_detect_size > 0.5) this->status.parameters.stop_status = 0;
            }

        }

    } else {
        pre_ornt_data = ornt_data;
        pre_mag_data = mag_data;
    }
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
 * @param gps_data, gps(lng,lat,alt,accuracy,speed,bearing,t)
 * @param ins_distance
 */
void Location::AutoAdjustTFactor(Eigen::VectorXd &gps_data, double ins_distance) {

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
//        double gps_dist1 = gps_queue(0,4) * ((gps_queue(1, 6) - gps_queue(0, 6)) / 1000.0) + gps_queue(1, 4) * ((gps_queue(2, 6) - gps_queue(1, 6)) / 1000.0);
        double gps_dist = gps.CalDistance(lng1, lat1, lng2, lat2) + gps.CalDistance(lng2, lat2, lng3, lat3);
        double ins_dist = ins_move_dist(1) + ins_move_dist(2);
        double deltaT = (gps_queue(2, 6) - gps_queue(0, 6)) / 1000.0;
        if (gps_dist != 0.0 && ins_dist != 0.0 && deltaT != 0.0) {
//            (*status).parameters.move_t_factor *= sqrt(gps_dist / ins_dist);
            this->status.parameters.move_t_factor = sqrt(gps_dist / ins_dist);
//            std::string log_msg = std::to_string(gps_dist) + " " + std::to_string(gps_dist1) + " "
//                                  + std::to_string(ins_dist) + " "
//                                  + std::to_string(deltaT) + " " + std::to_string(sqrt(gps_dist / ins_dist)) + " "
//                                  + std::to_string((*status).parameters.move_t_factor);
//            std::cout << log_msg << std::endl;
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
 * 惯导运动衰减因子,随时间及方位角变化而衰减
 */
void Location::AutoAdjustMovingFactor() {
    static Vector2d ornt_queue;
    static int cnt = 0;

    if (cnt <= 1) {
        ornt_queue(cnt) = this->status.attitude.yaw;
        cnt += 1;
    } else {
        // 更新队列数据
        ornt_queue(0) = ornt_queue(1);
        ornt_queue(1) = this->status.attitude.yaw;
        // 计算角度变化
        double angle_factor;
        if (ornt_queue(0) < 90.0 && ornt_queue(1) > 270.0) {
            angle_factor = ornt_queue(0) + 360.0 - ornt_queue(1);
        } else if (ornt_queue(1) < 90.0 && ornt_queue(0) > 270.0) {
            angle_factor = ornt_queue(1) + 360.0 - ornt_queue(0);
        } else {
            angle_factor = abs(ornt_queue(1) - ornt_queue(0));
        }

        int gap_time = int(this->status.parameters.Hz * this->status.parameters.least_gap_time_for_using_road);
        int ins_move_factor = this->status.parameters.ins_count % int(this->status.parameters.Hz);

        if (this->status.parameters.ins_count > gap_time &&
            (ins_move_factor == 0 || angle_factor >= this->status.parameters.accepted_max_diff_change_range)) {
            this->status.parameters.move_t_factor *= this->status.parameters.move_decay;
        }
//        std::cout << (*status).parameters.move_t_factor << std::endl;
    }
}

/**
 * 方向传感器和GPS,道路方向差值修正
 *
 * @param gps_data, gps(lng,lat,alt,accuracy,speed,bearing,t)
 * @param ornt_data, v(x,y,z)
 * @param road_data, 道路方向数据,包含距离下个路口距离,和当前点的瞬时方向,以及当前道路类型编码, v(distance, heading, code)
 */
void Location::UpdateZaxisWithGPSAndRoad(Eigen::VectorXd &gps_data, Eigen::Vector3d &ornt_data,
                                         Eigen::Vector3d &road_data) {
    static MatrixXd gps_queue(this->status.parameters.queue_gps_ornt, 7);
    static MatrixXd ornt_queue(this->status.parameters.queue_gps_ornt, 3);
    static MatrixXd road_queue(this->status.parameters.queue_gps_ornt, 3);
    static int gps_ornt_cnt = 0;
    static int road_ornt_cnt = 0;

    if (gps_ornt_cnt < this->status.parameters.queue_gps_ornt ||
        road_ornt_cnt < this->status.parameters.queue_gps_ornt) {
//    if (gps_ornt_cnt < (*status).parameters.queue_gps_ornt) {
        // gps方向队列
        if (gps_ornt_cnt < this->status.parameters.queue_gps_ornt &&
            gps_data(4) > this->status.parameters.gps_static_speed_threshold) {
            gps_queue.row(gps_ornt_cnt) = gps_data;
            ornt_queue.row(gps_ornt_cnt) = ornt_data;
            gps_ornt_cnt += 1;
        }
        // 道路方向队列
        if (road_ornt_cnt < this->status.parameters.queue_gps_ornt && road_data(0) != 0.0 && road_data(1) != 0.0) {
            road_queue.row(road_ornt_cnt) = road_data;
            road_ornt_cnt += 1;
        }
    } else {
        if (gps_data(0) == 0.0 && gps_data(1) == 0.0) {
            if (this->status.parameters.ins_count >
                this->status.parameters.Hz * this->status.parameters.least_gap_time_for_using_road) {
                // 没gps信号, 且超过一定时间后, 采用道路方向做修正
                VectorXd road_bearing = road_queue.col(1);
                VectorXd ornt_bearing = ornt_queue.col(2);
                double diff_road_ornt = (road_bearing - ornt_bearing).mean();
                // TODO: 当距下个路口一定距离同时方向差别在一定范围内,采用道路方向修正指南针方向
                this->status.parameters.diff_gps_ornt = diff_road_ornt;
//                std::cout << road_bearing(0) << " " << ornt_bearing(0) << std::endl;
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
                this->status.parameters.diff_gps_ornt = diff_gps_ornt;
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
            this->status.parameters.diff_gps_ornt = diff_gps_ornt;
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
        if (gps_data(0) != 0.0 && gps_data(1) != 0.0 && gps_data(4) > this->status.parameters.gps_static_speed_threshold) {
            for (int i = 0; i < this->status.parameters.queue_gps_ornt - 1; i++) {
                gps_queue.row(i) = gps_queue.row(i + 1);
            }
            gps_queue.row(this->status.parameters.queue_gps_ornt - 1) = gps_data;
        }

        // 更新道路数据
        if (road_data(0) != 0.0 || road_data(1) != 0.0) {
            for (int i = 0; i < this->status.parameters.queue_gps_ornt - 1; i++) {
                road_queue.row(i) = road_queue.row(i + 1);
            }
            road_queue.row(this->status.parameters.queue_gps_ornt - 1) = road_data;
        }

        // 更新指南针数据
        for (int i = 0; i < this->status.parameters.queue_gps_ornt - 1; i++) {
            ornt_queue.row(i) = ornt_queue.row(i + 1);
        }
        ornt_queue.row(this->status.parameters.queue_gps_ornt - 1) = ornt_data;

    }
//    std::cout << gps_queue(0,5) << " " << road_queue(0,1) << " " << ornt_queue(0,2) << " "
//              <<  (*status).parameters.diff_gps_ornt << std::endl;
}

/**
 * 方向传感器和GPS方向差值修正
 *
 * @param gps_data, gps(lng,lat,alt,accuracy,speed,bearing,t)
 * @param ornt_data, v(x,y,z)
 */
void Location::UpdateZaxisWithGPS(Eigen::VectorXd &gps_data, Eigen::Vector3d &ornt_data) {
    static MatrixXd gps_queue(this->status.parameters.queue_gps_ornt, 7);
    static MatrixXd ornt_queue(this->status.parameters.queue_gps_ornt, 3);
    static int gps_ornt_cnt = 0;

    if (gps_ornt_cnt < this->status.parameters.queue_gps_ornt) {
        // gps方向队列
        if (gps_data(4) > this->status.parameters.gps_static_speed_threshold) {
            gps_queue.row(gps_ornt_cnt) = gps_data;
            ornt_queue.row(gps_ornt_cnt) = ornt_data;
            gps_ornt_cnt += 1;
        }
    } else {

        // 更新GPS数据
        if (gps_data(4) > this->status.parameters.gps_static_speed_threshold) {
            for (int i = 0; i < this->status.parameters.queue_gps_ornt - 1; i++) {
                gps_queue.row(i) = gps_queue.row(i + 1);
            }
            gps_queue.row(this->status.parameters.queue_gps_ornt - 1) = gps_data;
        }

        // 更新指南针数据
        for (int i = 0; i < this->status.parameters.queue_gps_ornt - 1; i++) {
            ornt_queue.row(i) = ornt_queue.row(i + 1);
        }
        ornt_queue.row(this->status.parameters.queue_gps_ornt - 1) = ornt_data;

        // 有gps信号时尽量采用gps信号做辅助修正
        VectorXd gps_bearing = gps_queue.col(5);
        VectorXd ornt_bearing = ornt_queue.col(2);
        double diff_gps_ornt = (gps_bearing - ornt_bearing).mean();
        this->status.parameters.diff_gps_ornt = diff_gps_ornt;
        // 用此时有效的GPS方向更新道路差值
        this->status.parameters.diff_road_ornt = diff_gps_ornt;

    }
}

/**
 * 方向传感器和道路方向差值修正
 *
 * @param ornt_data, v(x,y,z)
 * @param road_data, 道路方向数据,包含距离下个路口距离,和当前点的瞬时方向,以及当前道路类型编码, v(distance, heading, code)
 */
void
Location::UpdateZaxisWithRoad(Eigen::Vector3d &ornt_data, Eigen::Vector3d &road_data) {
    static MatrixXd ornt_queue(this->status.parameters.queue_gps_ornt, 3);
    static MatrixXd road_queue(this->status.parameters.queue_gps_ornt, 3);
    static int road_ornt_cnt = 0;

    if (road_ornt_cnt < this->status.parameters.queue_gps_ornt) {
        // 道路方向队列
        if (road_data(0) != 0.0 || road_data(1) != 0.0) {
            road_queue.row(road_ornt_cnt) = road_data;
            road_ornt_cnt += 1;
        }
    } else {
        // 更新道路数据
        if (road_data(0) != 0.0 || road_data(1) != 0.0) {
            for (int i = 0; i < this->status.parameters.queue_gps_ornt - 1; i++) {
                road_queue.row(i) = road_queue.row(i + 1);
            }
            road_queue.row(this->status.parameters.queue_gps_ornt - 1) = road_data;
        }

        // 更新指南针数据
        for (int i = 0; i < this->status.parameters.queue_gps_ornt - 1; i++) {
            ornt_queue.row(i) = ornt_queue.row(i + 1);
        }
        ornt_queue.row(this->status.parameters.queue_gps_ornt - 1) = ornt_data;

        if (this->status.parameters.ins_count >
            this->status.parameters.Hz * this->status.parameters.least_gap_time_for_using_road) {
            // 没gps信号, 且超过一定时间后, 采用道路方向做修正
            VectorXd road_bearing = road_queue.col(1);
            VectorXd ornt_bearing = ornt_queue.col(2);
            double diff_road_ornt = (road_bearing - ornt_bearing).mean();
            // TODO: 当距下个路口一定距离同时方向差别在一定范围内,采用道路方向修正指南针方向
            this->status.parameters.diff_road_ornt = diff_road_ornt;
        } else {
            // 没信号,但是没有超过一定时间间隔, 仍用当前指南针和上个GPS点修正
            this->status.parameters.diff_road_ornt = this->status.parameters.diff_gps_ornt;
        }

    }
}

/**
 *  判断道路方向变化幅度是否与指南针幅度一致
 *
 * @param ornt_data, v(x,y,z)
 * @param road_data, 道路方向数据,包含距离下个路口距离,和当前点的瞬时方向,以及当前道路类型编码, v(distance, heading, code)
 * @return
 */
bool Location::IsRoadCompassSameRange(Vector3d &ornt_data, Vector3d &road_data) {
    static MatrixXd ornt_queue(this->status.parameters.queue_road_ornt_len, 3);
    static MatrixXd road_queue(this->status.parameters.queue_road_ornt_len, 3);
    static int road_ornt_cnt = 0;

    bool res = true;
    if (road_ornt_cnt < this->status.parameters.queue_road_ornt_len) {

        // 道路方向队列
        if (road_data(0) != 0.0 || road_data(1) != 0.0) {
            if (road_ornt_cnt == 0) {
                road_queue.row(road_ornt_cnt) = road_data;
                ornt_queue.row(road_ornt_cnt) = ornt_data;
                road_ornt_cnt += 1;
            } else {
                if (road_queue(road_ornt_cnt - 1, 1) != road_data(1)
                    || road_queue(road_ornt_cnt - 1, 0) != road_data(0)) {
                    road_queue.row(road_ornt_cnt) = road_data;
                    ornt_queue.row(road_ornt_cnt) = ornt_data;
                    road_ornt_cnt += 1;
                }
            }
        }

    } else {

        // 更新道路数据
        if ((road_data(0) != 0.0 || road_data(1) != 0.0) &&
            (road_queue(road_ornt_cnt - 1, 1) != road_data(1)
             || road_queue(road_ornt_cnt - 1, 0) != road_data(0))) {
            for (int i = 0; i < this->status.parameters.queue_road_ornt_len - 1; i++) {
                road_queue.row(i) = road_queue.row(i + 1);
            }
            road_queue.row(this->status.parameters.queue_road_ornt_len - 1) = road_data;

            // 更新指南针数据
            for (int i = 0; i < this->status.parameters.queue_road_ornt_len - 1; i++) {
                ornt_queue.row(i) = ornt_queue.row(i + 1);
            }
            ornt_queue.row(this->status.parameters.queue_road_ornt_len - 1) = ornt_data;
        }

        VectorXd road_bearing = road_queue.col(1);
        VectorXd ornt_bearing = ornt_queue.col(2);
        VectorXd road_diff(this->status.parameters.queue_road_ornt_len - 1);
        VectorXd ornt_diff(this->status.parameters.queue_road_ornt_len - 1);
        LPF lpf;
        for (int i = 0; i <= (this->status.parameters.queue_road_ornt_len - 2); ++i) {
            std::string road_flag = lpf.JudgeOrientation(road_bearing(i)) + lpf.JudgeOrientation(road_bearing(i + 1));
            std::string ornt_flag = lpf.JudgeOrientation(ornt_bearing(i)) + lpf.JudgeOrientation(ornt_bearing(i + 1));
            // 处理0/360跳点问题
            if (road_flag == "14") {
                road_diff(i) = road_bearing(i) - (road_bearing(i + 1) + 360.0);
            } else if (road_flag == "41") {
                road_diff(i) = (road_bearing(i + 1) + 360.0) - road_bearing(i);
            } else {
                road_diff(i) = road_bearing(i + 1) - road_bearing(i);
            }
            if (ornt_flag == "14") {
                ornt_diff(i) = ornt_bearing(i) - (ornt_bearing(i + 1) + 360.0);
            } else if (ornt_flag == "41") {
                ornt_diff(i) = (ornt_bearing(i + 1) + 360.0) - ornt_bearing(i);
            } else {
                ornt_diff(i) = ornt_bearing(i + 1) - ornt_bearing(i);
            }

        }

        for (int i = 0; i <= this->status.parameters.queue_road_ornt_len - 2; i++) {
            if (abs(road_diff(i)) < this->status.parameters.accepted_change_range) {
                res = res && abs((road_diff(i) - ornt_diff(i))) < this->status.parameters.accepted_change_range;
            } else {
                res = res && abs(road_diff(i) - ornt_diff(i)) < this->status.parameters.accepted_max_diff_change_range;
            }
//            std::cout << (road_diff(i) < (*status).parameters.accepted_change_range)
//                      << " " << road_diff(i) << " " << road_bearing(i) << " "  << road_bearing(i + 1)
//                      << " " << ornt_diff(i) << " " << ornt_bearing(i) << " " << ornt_bearing(i + 1)
//                      << " " << res << std::endl;
        }
    }
    return res;
}

/**
 * 判断当前是否正处于重新规划中
 *
 * @param status
 * @param ornt_data, v(x,y,z)
 * @param road_data, 道路方向数据,包含距离下个路口距离,和当前点的瞬时方向,以及当前道路类型编码, v(distance, heading, code)
 * @return
 */
bool Location::IsRouting(Eigen::Vector3d &ornt_data, Eigen::Vector3d &road_data) {
    static bool routing = false;
    static int cnt = 0;

    if (road_data(0) == 0.0 && road_data(1) == 0.0) routing = true;
    if (routing) cnt += 1;

    if (cnt > (this->status.parameters.routing_time * this->status.parameters.Hz)) {
        cnt = 0;
        routing = false;
    }
    return routing;

}

/**
 * 判断是否偏离航线
 *
 * @param ornt_data, v(x,y,z)
 * @param road_data, 道路方向数据,包含距离下个路口距离,和当前点的瞬时方向,以及当前道路类型编码, v(distance, heading, code)
 * @return
 */
bool Location::IsOffCourse(Eigen::Vector3d &ornt_data, Eigen::Vector3d &road_data) {

    static VectorXd is_near_cross_queue(this->status.parameters.off_course_data_queue);
    static VectorXd is_same_change_queue(this->status.parameters.off_course_data_queue);
    static int cnt = 0;
    static bool routing = false;

    // 偏航标志
    bool offCourseFlag = false;

    // 判断是否以及行走到路口一定范围内
    bool is_near_cross =
            this->status.parameters.dist_from_pre_cross < this->status.parameters.min_dist_from_pre_cross ||
            this->status.parameters.dist_to_next_cross < this->status.parameters.min_dist_to_next_cross;

    // 判断指南针跟道路的方向变化是否一直
    bool is_same_change = IsRoadCompassSameRange(ornt_data, road_data);

    if (cnt < this->status.parameters.off_course_data_queue) {
        is_near_cross_queue(cnt) = is_near_cross;
        is_same_change_queue(cnt) = is_same_change;
        cnt += 1;
    } else {

        if (is_near_cross_queue(0)) {
            // 在路口发生偏航的时候
            bool res = true;
            // 连续一个窗口内都判断是否偏航
            for (int i = 0; i < this->status.parameters.off_course_data_queue; ++i) {
                res = res && is_near_cross_queue(i) && !is_same_change_queue(i);
            }
            offCourseFlag = res;
        }

        // 队列先进先出
        for (int k = 0; k < this->status.parameters.off_course_data_queue - 1; ++k) {
            is_near_cross_queue(k) = is_near_cross_queue(k + 1);
            is_same_change_queue(k) = is_same_change_queue(k + 1);
        }
        is_near_cross_queue(this->status.parameters.off_course_data_queue - 1) = is_near_cross;
        is_same_change_queue(this->status.parameters.off_course_data_queue - 1) = is_same_change;

    }

    // 重新规划,重置flag.
    if (road_data(0) == 0.0 && road_data(1) == 0.0) routing = true;
    if (routing && (road_data(0) != 0.0 || road_data(1) != 0.0)) {
        offCourseFlag = false;
        routing = false;
    }

    return offCourseFlag;
}

/**
 * 更新道路状态
 * @param road_data, v(distance, bearing, code)
 */
void Location::UpdateRoadType(Eigen::Vector3d &road_data) {
    static double jump_point = 0.0;

    if (road_data(0) != 0.0 || road_data(1) != 0.0) {
        this->status.parameters.road_type = road_data(2);

        if (road_data(0) > this->status.parameters.dist_to_next_cross) {
            jump_point = road_data(0);
            this->status.parameters.dist_from_pre_cross = 0.0;
        } else {
            if (jump_point > road_data(0)) {
                this->status.parameters.dist_from_pre_cross = jump_point - road_data(0);
            }
        }
        this->status.parameters.dist_to_next_cross = road_data(0);
    } else {
        jump_point = 0.0;
        this->status.parameters.road_type = 0.0;
        this->status.parameters.dist_to_next_cross = 100000.0;
        this->status.parameters.dist_from_pre_cross = 100000.0;
    }

}

/**
 * 获取当前输入GPS点状态, 需在调用`PredictCurrentPosition`方法后调用.
 * @return true 为可用, false 不可用
 */
bool Location::GetCurentGPSStatus() {
    return status.parameters.is_current_gps_valid;
}

//Eigen::VectorXd Location::GPSJumpPointCompensate(routing::Status *status, Eigen::VectorXd &gps_bearing_queue) {
//
//
//}