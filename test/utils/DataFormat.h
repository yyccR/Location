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

    std::vector<std::string> getAllfiles(std::string &dir);

    void writeCSVs();

    void writeCSV(std::string file_name, MatrixXd &acc, MatrixXd &g_data, MatrixXd &gyro,
                  MatrixXd &mag, MatrixXd &ornt_data, MatrixXd &gps_data);

    void readCSV(std::string file_name, MatrixXd &gyro, MatrixXd &acc, MatrixXd &mag,
                 MatrixXd &gps_data, MatrixXd &g_data, MatrixXd &ornt_data, MatrixXd &road_data);

    void split(const string &s, vector<string> &v, const string &c);

    void readCaliData(MatrixXd &gyro, MatrixXd &acc, MatrixXd &mag);
};


#endif //LOCATION_DATAFORMAT_H
