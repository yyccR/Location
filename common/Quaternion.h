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
    // rewrite the operator '*' for quaternion and vector multiplying.
    Vector3 operator*(const Vector3 &vec) const;


    // get quaternions from axis-angle.
    void FromAxis(const Vector3 &v, double angle);
    // get quaternions from Euler-angle.
    void FromEuler(double pitch, double yaw, double roll);

    // convert quaternion into axis-angle.
    void GetAxisAngle(Vector3 *axis, float *angle);

};


#endif //LOCATION_QUATERNION_H
