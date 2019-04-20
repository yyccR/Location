//
// Created by yangcheng on 2018/12/24.
//

#include <iostream>
#include "GPS.h"
#include "../math/KalmanFilter.h"

using namespace Eigen;
using namespace routing;

/**
 * 给定起点经度,纬度,距离,方向, 计算终点的经纬度.
 *
 * @param startLng 起点经度
 * @param startLat 起点纬度
 * @param distance 距离
 * @param heading 方向（与正北夹角）,角度
 * @return
 */
Eigen::Vector2d GPS::CalDestination(double &startLng, double &startLat, double &distance, double &heading) {
    Vector2d nextLngLat;
    double R = 6378.137 * 1000.0;
    double rad_lng = startLng / 180.0 * M_PI;
    double rad_lat = startLat / 180.0 * M_PI;
    double heading_rad = heading / 180.0 * M_PI;
    double lat = asin(sin(rad_lat) * cos(distance / R) + cos(rad_lat) * sin(distance / R) * cos(heading_rad));
    double lng = rad_lng + atan2(sin(heading_rad) * sin(distance / R) * cos(rad_lat),
                                 cos(distance / R) - sin(rad_lat) * sin(lat));
    nextLngLat(0) = lng / M_PI * 180.0;
    nextLngLat(1) = lat / M_PI * 180.0;
    return nextLngLat;
}

/**
 * 根据GPS速度,方向 计算正北,正东方向速度.
 *
 * @param status
 * @param velocity
 * @param bearing
 */
void GPS::UpdateVelocity(Status *status, double &velocity, double &bearing) {
    double bearing_rad = bearing / 180.0 * M_PI;
    double v_north = velocity * cos(bearing_rad);
    double v_east = velocity * sin(bearing_rad);
    (*status).velocity.v_x = v_north;
    (*status).velocity.v_y = v_east;
}

/**
 * 计算两个经纬度点之间的距离.
 *
 * @param startLng 起点经度
 * @param startLat 起点纬度
 * @param endLng 终点经度
 * @param endLat 终点纬度
 * @return
 */
double GPS::CalDistance(double &startLng, double &startLat, double &endLng, double &endLat) {

    double startLng_rad = startLng / 180.0 * M_PI;
    double startLat_rad = startLat / 180.0 * M_PI;
    double endLng_rad = endLng / 180.0 * M_PI;
    double endLat_rad = endLat / 180.0 * M_PI;


    double a = pow(sin((endLat_rad - startLat_rad) / 2.0), 2.0) +
               cos(startLat_rad) * cos(endLat_rad) * pow(sin((endLng_rad - startLng_rad) / 2.0), 2.0);
    double c = 2 * asin(sqrt(a));
    return c * 6378.137 * 1000.0;
}

/**
 * 计算当前kalman输入所需gps状态
 *
 * @param gps_data
 * @return vector4d(lng,lat,v_east,v_north)
 */
Eigen::Vector4d GPS::CalcState(Eigen::VectorXd &gps_data) {
    // 根据当前gps角度计算两个分速度
    double bearing_rad = gps_data(5) / 180.0 * M_PI;
    double v_north = gps_data(4) * cos(bearing_rad);
    double v_east = gps_data(4) * sin(bearing_rad);

    Vector4d cur_state(gps_data(0), gps_data(1), v_east, v_north);
    return cur_state;
}

/**
 * 判断GPS点是否符合GPS历史轨迹
 *
 * @param status
 * @param gps_data, gps(lng,lat,alt,accuracy,speed,bearing,t)
 * @return
 */
bool GPS::IsGPSBelongToTrack(routing::Status *status, Eigen::VectorXd &gps_data) {

    bool result = true;
    static MatrixXd gps_queue((*status).parameters.gps_track_len, 7);
    static int cnt = 0;
    static int ignore_in_tunnel = 0;

//    std::cout << "IsGPSBelongToTrack in " << std::endl;
    if (cnt < (*status).parameters.gps_track_len) {
        if (cnt >= 1) {
            // 初步判断该点是否时间在允许的范围内
            // TODO: 可以做更进一步判断该点是否合理能被加入数据做滤波轨迹
            double time_diff1 = (gps_data(6) - gps_queue(cnt - 1, 6)) / 1000.0;
            if (time_diff1 < (*status).parameters.gps_max_gap_time && time_diff1 != 0.0) {
                gps_queue.row(cnt) = gps_data;
                cnt += 1;
            } else {
                cnt = 0;
            }
        } else {
            gps_queue.row(cnt) = gps_data;
            cnt += 1;
        }
    } else {

        // 初始化kalman
        VectorXd init_state = gps_queue.row(0);
        Vector4d cur_state = CalcState(init_state);
        KalmanFilter kalmanFilter(cur_state);

        // 滤波计算
        for (int i = 1; i < (*status).parameters.gps_track_len; ++i) {
            VectorXd origin_gps = gps_queue.row(i);
            Vector4d measurement_state = CalcState(origin_gps);
            double delta_t = (gps_queue(i, 6) - gps_queue(i - 1, 6)) / 1000.0;
            kalmanFilter.SetF(delta_t);
            kalmanFilter.UpdateState(measurement_state);
        }

        // 计算比较当前预测与实际测量的差距
        VectorXd predict_state = kalmanFilter.PredictState();
        double start_lng = predict_state(0);
        double start_lat = predict_state(1);
        double end_lng = gps_data(0);
        double end_lat = gps_data(1);
        double cur_error = CalDistance(start_lng, start_lat, end_lng, end_lat);
        double time_diff2 = (gps_data(6) - gps_queue.row((*status).parameters.gps_track_len - 1)(6)) / 1000.0;

//        std::cout << "cur_error <= (*status).parameters.weak_gps " << cur_error << std::endl;
        if (cur_error <= (*status).parameters.weak_gps) {
            // 误差在可接受的范围
            for (int i = 0; i < (*status).parameters.gps_track_len - 1; i++) {
                gps_queue.row(i) = gps_queue.row(i + 1);
            }
            gps_queue.row((*status).parameters.gps_track_len - 1) = gps_data;
        } else {

            if ((*status).parameters.road_type == 1.0){
                // 误差不可接受, 同时处于隧道状态, 则不考虑时间间隔,同时不拿下个点作为初始了,因为下个点可能是仍在隧道,但有次数限制
                if(ignore_in_tunnel < (*status).parameters.max_ignore_in_tunnel){
                    ignore_in_tunnel += 1;
                }else{
                    cnt = 0;
                    ignore_in_tunnel = 0;
                }
                result = false;
            }else{
                // 误差不可接受,但不处于隧道状态,但是时间间隔超出了允许的间隔范围,则认为该点已超出预估能力范围
                // 误差不可接受,但不处于隧道状态,时间间隔也没有超出了允许的间隔范围,则认为改点是误差点
                result = time_diff2 > (*status).parameters.gps_max_gap_time;
                cnt = 0;
            }
        }
    }
    return result;
}

/**
 * 根据当前输入以及状态判断采用GPS还是INS
 *
 * @param status
 * @param gps_data, GPS原始数据, gps(lng,lat,alt,accuracy,speed,bearing,t)
 * @return gps是否有效
 */
bool GPS::IsGPSValid(Status *status, VectorXd *gps_data) {

    // 精度是否足够
    bool is_gps_accuracy = (*gps_data)(3) <= (*status).parameters.weak_gps;

    // gps是否为空
    bool is_gps_not_null = ((*gps_data)(0) != 0.0 && (*gps_data)(1) != 0.0);

    // gps运动距离是否过大,判断是否相同时间间隔GPS移动与惯导相差不大,用于判断该点是否被采用
    double start_lng = (*status).position.lng;
    double start_lat = (*status).position.lat;
    double end_Lng = (*gps_data)(0);
    double end_Lat = (*gps_data)(1);
    double gps_move_dist = CalDistance(start_lng, start_lat, end_Lng, end_Lat);
    bool is_gps_move_accepted = ceil((*status).parameters.ins_count / 10.0 + 1.0) * gps_move_dist <
                                ceil((*status).parameters.ins_count / 10.0 + 1.0) *
                                (*status).parameters.move_distance_threshod;

    // 判断是否当前是导航初始状态
    bool is_gps_initializing = (*status).parameters.gps_count <= (*status).parameters.gps_init_threshold;

    // 判断当前GPS是否与上个GPS点同个时间戳
    bool is_gps_not_duplicated = (*gps_data)(6) != (*status).parameters.gps_pre_t;

    // 当GPS速度为0时,方向沿用上个GPS点方向
    if ((*gps_data)(4) == 0.0) {
        (*gps_data)(5) = (*status).parameters.gps_pre_bearing;
    }

    // 判断GPS高精情况下是否存在漂移,利用kalman根据历史轨迹进行滤波估计
    bool is_gps_belong_to_track = true;
    if (is_gps_accuracy && is_gps_not_null && is_gps_not_duplicated) {
        is_gps_belong_to_track = IsGPSBelongToTrack(status, (*gps_data));
    }

    // 当GPS在初始状态内时，处理为0.0的情况,gps(lng,lat,alt,accuracy,speed,bearing,t)
    if (is_gps_initializing && !is_gps_not_null) {
        (*gps_data)(0) = (*status).parameters.gps_pre_lng;
        (*gps_data)(1) = (*status).parameters.gps_pre_lat;
        (*gps_data)(2) = (*status).parameters.gps_pre_altitude;
        (*gps_data)(3) = (*status).parameters.gps_pre_accuracy;
        (*gps_data)(4) = (*status).parameters.gps_pre_speed;
        (*gps_data)(5) = (*status).parameters.gps_pre_bearing;
        (*gps_data)(6) = (*status).parameters.gps_pre_t;
    }

    // 当载体处于静止，则无论怎样都采用该GPS
    bool is_gps_static;
    if (is_gps_not_null && (*gps_data)(4) <= (*status).parameters.gps_static_speed_threshold) {
        is_gps_static = true;
        // 静止时候传感器采用静止因子
        (*status).parameters.t = 1.0 / ((*status).parameters.Hz * (*status).parameters.static_t_factor);
    } else {
        is_gps_static = false;
        // 运动时候传感器采用运动因子
//        (*status).parameters.t = 1.0 / ((*status).parameters.Hz * (*status).parameters.move_t_factor);
    }

    // 当前一个GPS点速度为0,当前GPS数据又为空的的时候,采用前一点的数据,gps(lng,lat,alt,accuracy,speed,bearing,t)
    bool is_gps_still_static;
    if (!is_gps_not_null && (*status).parameters.gps_pre_speed <= (*status).parameters.gps_static_speed_threshold) {
        (*gps_data)(0) = (*status).parameters.gps_pre_lng;
        (*gps_data)(1) = (*status).parameters.gps_pre_lat;
        (*gps_data)(2) = (*status).parameters.gps_pre_altitude;
        (*gps_data)(3) = (*status).parameters.gps_pre_accuracy;
        (*gps_data)(4) = (*status).parameters.gps_pre_speed;
        (*gps_data)(5) = (*status).parameters.gps_pre_bearing;
        (*gps_data)(6) = (*status).parameters.gps_pre_t;
        is_gps_still_static = true;
        // 静止时候传感器采用静止因子
        (*status).parameters.t = 1.0 / ((*status).parameters.Hz * (*status).parameters.static_t_factor);
    } else {
        is_gps_still_static = false;
        // 运动时候传感器采用运动因子
//        (*status).parameters.t = 1.0 / ((*status).parameters.Hz * (*status).parameters.move_t_factor);
    }

    return (is_gps_accuracy && is_gps_not_null && is_gps_move_accepted && is_gps_not_duplicated &&
            is_gps_belong_to_track)
           || is_gps_initializing || is_gps_static || is_gps_still_static;
}