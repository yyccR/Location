//
// Created by yangcheng on 2019/7/1.
//

#ifndef LOCATION_STOPDETECTION_H
#define LOCATION_STOPDETECTION_H

#include <Eigen/Dense>

class StopDetection {
public:

    StopDetection();
    explicit StopDetection(std::string *model);
    ~StopDetection();

    virtual bool IsStopping(Eigen::VectorXd &data) const;
};


#endif //LOCATION_STOPDETECTION_H
