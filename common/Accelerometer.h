//
// Created by yangcheng on 2018/11/28.
//
#include "Quaternion.h"

#ifndef LOCATION_ACCELEROMETER_H
#define LOCATION_ACCELEROMETER_H

class Accelerometer {

    // accelerate data
    double x, y, z;

    Accelerometer(double x, double y, double z);
    virtual ~Accelerometer();

    // rotate the accelerate data into .
    void Rotate(const Quaternion &quaternion, const Quaternion &quaternion_inv);


};


#endif //LOCATION_ACCELEROMETER_H
