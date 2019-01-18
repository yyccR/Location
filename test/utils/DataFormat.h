//
// Created by yangcheng on 2019/1/17.
//

#ifndef LOCATION_DATAFORMAT_H
#define LOCATION_DATAFORMAT_H

#include "vector"
#include "string"
#include "Eigen/Dense"

using namespace std;
using namespace Eigen;

class DataFormat {
public:

    void readCSV(MatrixXd &gyro, MatrixXd &acc, MatrixXd &mag);

    void split(const string &s, vector<string> &v, const string &c);
};


#endif //LOCATION_DATAFORMAT_H
