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

    void readCSV(MatrixXd &gyro, MatrixXd &acc, MatrixXd &mag, MatrixXd &gps_data, MatrixXd &g_data, VectorXd &heading);

    void split(const string &s, vector<string> &v, const string &c);

    void readCaliData(MatrixXd &gyro, MatrixXd &acc, MatrixXd &mag);
};


#endif //LOCATION_DATAFORMAT_H
