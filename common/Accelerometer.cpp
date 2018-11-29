//
// Created by yangcheng on 2018/11/28.
//

#include "Accelerometer.h"
#include "Quaternion.h"


Accelerometer::Accelerometer(double x, double y, double z) {
    this->x = x;
    this->y = y;
    this->z = z;
}

Accelerometer::~Accelerometer() {}

Accelerometer Accelerometer::Rotate(const Quaternion &quaternion, const Quaternion &quaternion_inv) {
    Quaternion QA(0, this->x, this->y, this->z);
    Quaternion qleft = quaternion * QA;
    Quaternion qright = qleft * quaternion_inv;
    return Accelerometer(qright.x, qright.y, qright.z);
}

void Accelerometer::Format() {

}

