//
// Created by yangcheng on 2019/1/17.
//

#ifndef LOCATION_TESTLOCATION_H
#define LOCATION_TESTLOCATION_H

#include "../location/Location.h"


class TestLocation {
public:

    void testLocation(Eigen::MatrixXd &gyro_data, Eigen::MatrixXd &acc_data, Eigen::MatrixXd &mag_data,
                      Eigen::MatrixXd &gps_data, Eigen::MatrixXd &g_data, Eigen::VectorXd &heading);
};


#endif //LOCATION_TESTLOCATION_H
