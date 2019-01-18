//
// Created by yangcheng on 2019/1/17.
//

#ifndef LOCATION_TESTLOCATION_H
#define LOCATION_TESTLOCATION_H

#include "../location/Location.h"

class TestLocation {
public:

    void testLocation(MatrixXd &gyro_data, MatrixXd &acc_data, MatrixXd &mag_data);
};


#endif //LOCATION_TESTLOCATION_H
