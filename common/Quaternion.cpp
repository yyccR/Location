//
// Created by yangcheng on 2018/11/27.
//

#include <math.h>
#include "Quaternion.h"

#define TOLERANCE 0.00001
#define PIOVER180 1.0

void Quaternion::Normalise() {

    double norm2 = this->w * this->w + this->x * this->x + this->y * this->y + this->z * this->z;
    if (norm2 != 0.0 && (fabs(mag2 - 1.0) > TOLERANCE)) {
        float norm = sqrt(norm2);
        this->w /= norm;
        this->x /= norm;
        this->y /= norm;
        this->z /= norm;
    }
}

Quaternion Quaternion::GetConjugate() {

    return Quaternion(this->w, -this->x, -this->y, -this->z);
}

Quaternion Quaternion::operator*(const Quaternion &quaternion) const {

    return Quaternion(
            this->w * quaternion.w -this->x * quaternion.x - this->y * quaternion.y - this->z * quaternion.z,
            this->w * quaternion.x +this->x * quaternion.w + this->y * quaternion.z - this->z * quaternion.y,
            this->w * quaternion.y -this->x * quaternion.z + this->y * quaternion.w + this->z * quaternion.x,
            this->w * quaternion.z +this->x * quaternion.y - this->y * quaternion.x + this->z * quaternion.w
    );
}

void Quaternion::FromAxis(const Vector3 &v, double angle) {

}


void Quaternion::FromEuler(double pitch, double yaw, double roll) {

    double p = pitch * PIOVER180 / 2.0;
    double y = yaw * PIOVER180 / 2.0;
    double r = roll * PIOVER180 / 2.0;

    double sinp = sin(p);
    double siny = sin(y);
    double sinr = sin(r);
    double cosp = cos(p);
    double cosy = cos(y);
    double cosr = cos(r);

    this->w = cosr * cosp * cosy + sinr * sinp * siny;
    this->x = sinr * cosp * cosy - cosr * sinp * siny;
    this->y = cosr * sinp * cosy + sinr * cosp * siny;
    this->z = cosr * cosp * siny - sinr * sinp * cosy;

    this->Normalise();
}