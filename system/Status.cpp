//
// Created by yangcheng on 2019/1/14.
//

#include "Status.h"

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