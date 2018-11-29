//
// Created by yangcheng on 2018/11/27.
//

#ifndef LOCATION_QUATERNION_H
#define LOCATION_QUATERNION_H


class Quaternion {
public:

    // quaternion scale value
    double w;
    // quaternion vector.
    double x, y, z;

    Quaternion(double w, double x, double y, double z);
    virtual ~Quaternion();

    // normalise the quaternion.
    void Normalise();

    // get conjugate quaternion.
    Quaternion GetConjugate();

    // rewrite the operator '+' for quaternion addition.
    Quaternion operator+(const Quaternion &quaternion) const;
    // rewrite the operator '*' for quaternion multiplying.
    Quaternion operator*(const Quaternion &quaternion) const;

    // get quaternions from Euler-angle.
    void FromEuler(double pitch, double yaw, double roll);

};


#endif //LOCATION_QUATERNION_H
