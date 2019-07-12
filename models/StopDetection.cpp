//
// Created by yangcheng on 2019/7/1.
//

#include "StopDetection.h"

StopDetection::StopDetection() {};

StopDetection::StopDetection(std::string *model) {};

StopDetection::~StopDetection() {};

bool StopDetection::IsStopping(Eigen::VectorXd &data) const { return false; };