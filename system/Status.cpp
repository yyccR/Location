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
    Vector3d err(0.0,0.0,0.0);
    this->parameters.err = err;
    this->parameters.ki = 0.05;
    this->parameters.kp = 10.0;
    this->parameters.halfT = 1 / 20.0;
    this->parameters.g = 9.805567;
    this->parameters.mag = 157.44;
    this->parameters.t = 1 / 2.5;
    this->parameters.move_distance_threshod = 100.0;
    this->parameters.ins_count = 0;
    this->parameters.gps_count = 0;
    this->parameters.gps_init_threshold = 180;
    this->parameters.gps_pre_t = 0.0;
}