//
// Created by yangcheng on 2019/1/14.
//

#include "Status.h"
#include "../math/Quaternions.h"
#include "../sensor/Gyroscope.h"
#include "../sensor/Accelerometer.h"
#include "../sensor/Magnetometer.h"
#include "Eigen/Dense"

using namespace Eigen;
using namespace routing;

Position Status::GetPosition() const {
    return this->position;
}

Attitude Status::GetAttitude() const {
    return this->attitude;
}

Velocity Status::GetVelocity() const {
    return this->velocity;
}

Parameters Status::GetParameters() const {
    return this->parameters;
}

void Status::Init() {

    this->gnssins.lng = 0.0;
    this->gnssins.lat = 0.0;
    this->gnssins.altitude = 0.0;
    this->gnssins.accuracy = 0.0;
    this->gnssins.speed = 0.0;
    this->gnssins.bearing = 0.0;

    this->velocity.v_x = 0.0;
    this->velocity.v_y = 0.0;
    this->velocity.v_z = 0.0;

    this->position.x = 0.0;
    this->position.y = 0.0;
    this->position.z = 0.0;
    this->position.lng = 0.0;
    this->position.lat = 0.0;
    this->position.altitude = 0.0;

    this->attitude.roll = 0.0;
    this->attitude.pitch = 0.0;
    this->attitude.yaw = 0.0;


    Quaternions quaternions;
    Vector3d init_euler_angle(this->attitude.roll, this->attitude.pitch, this->attitude.yaw);
    this->attitude.q_attitude = quaternions.GetQFromEuler(init_euler_angle);
    Vector3d gyro_coef(0.0,0.0,0.0);
    this->parameters.gyro_coef = gyro_coef;
    VectorXd acc_coef(6);
    acc_coef << 0.0,0.0,0.0,1.0,1.0,1.0;
    this->parameters.acc_coef = acc_coef;
    VectorXd mag_coef(6);
    mag_coef << 0.0,0.0,0.0,1.0,1.0,1.0;
    this->parameters.mag_coef = mag_coef;

    this->parameters.weak_gps = 100;
    this->parameters.gamma = 1.0;
    this->parameters.epsilon = 0.000001;
    this->parameters.max_step = 200;

    this->parameters.acc_a1 = 0.0;
    this->parameters.acc_a2 = 0.0;
    this->parameters.acc_b0 = 0.0;
    this->parameters.acc_hz = 5.0;
    this->parameters.acc_thres = 0.1;
    Vector3d acc(0.0,0.0,0.0);
    this->parameters.last_acc_data = acc;
    this->parameters.sec_last_acc_data = acc;
    this->parameters.ornt_hz = 1.5;
    Vector3d ornt(0.0,0.0,0.0);
    this->parameters.last_ornt_data = ornt;
    this->parameters.sec_last_ornt_data = ornt;

    Vector3d err(0.0,0.0,0.0);
    this->parameters.err = err;
    this->parameters.ki = 0.1;
    this->parameters.kp = 1.0;
    this->parameters.Hz = 20.0;
    this->parameters.halfT = 1.0 / (this->parameters.Hz * 2.0);
    this->parameters.static_t_factor = 1.0;
    this->parameters.move_t_factor = 1.0;
    this->parameters.t = 1.0 / (this->parameters.Hz * this->parameters.static_t_factor);
    this->parameters.move_decay = 0.995;

    this->parameters.move_distance_threshod = 500000.0;
    this->parameters.ins_count = 0;
    this->parameters.ins_dist = 0.0;
    this->parameters.max_ins_dist = 1000.0;

    this->parameters.least_gap_time_for_using_road = 2.5;
    this->parameters.queue_gps_ornt = 1;
    this->parameters.diff_gps_ornt = 0.0;
    this->parameters.diff_road_ornt = 0.0;
    this->parameters.queue_road_ornt_len = 3;
    this->parameters.accepted_change_range = 8.0;
    this->parameters.accepted_max_diff_change_range = 12.0;
    this->parameters.off_course_data_queue = 30;
    this->parameters.routing_time = 5;

    this->parameters.gps_static_speed_threshold = 2.0;
    this->parameters.gps_count = 0;
    this->parameters.gps_init_threshold = 10;
    this->parameters.gps_pre_lng = 0.0;
    this->parameters.gps_pre_lat = 0.0;
    this->parameters.gps_pre_t = 0.0;
    this->parameters.gps_pre_speed = 0.0;
    this->parameters.gps_pre_accuracy = 0.0;
    this->parameters.gps_pre_bearing = 0.0;
    this->parameters.gps_pre_altitude = 0.0;
    this->parameters.gps_track_len = 7;
    this->parameters.gps_max_gap_time = 5;

    this->parameters.shaking_threshold = 0.8;
    this->parameters.compass_queue_len = 20;
    this->parameters.compass_vaild_var_thres = 0.085;

    this->parameters.road_type = 0.0;
    this->parameters.max_ignore_in_common = 3;
    this->parameters.max_ignore_in_tunnel = 8;
    this->parameters.min_dist_to_next_cross = 50.0;
    this->parameters.min_dist_from_pre_cross = 150.0;
    this->parameters.dist_to_next_cross = 100000.0;
    this->parameters.dist_from_pre_cross = 100000.0;

    this->parameters.g = 9.805567;
    this->parameters.mag = 157.44;
    this->parameters.we = 7.2921158 / 100000.0;
    this->parameters.R = 6378137.0;
}