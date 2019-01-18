//
// Created by yangcheng on 2019/1/17.
//

#include "DataFormat.h"
#include "fstream"
#include "cassert"


using namespace std;
using namespace Eigen;

void DataFormat::readCSV(MatrixXd &gyro, MatrixXd &acc, MatrixXd &mag) {
    ifstream infile;
    infile.open("C:/Users/yangcheng/Desktop/GNSS惯性导航/data/IMU/GPS-IMU-Integration-master/data/Editing/EEWalk2.csv",
                ios::in);
    assert(infile.is_open());


    MatrixXd v;
    string s;
    int i = 0;
    while (getline(infile, s)) {

        vector<string> s_split;
        split(s, s_split, ",");

        // acc data.
        acc(i,0) = atof(s_split[0].c_str());
        acc(i,1) = atof(s_split[1].c_str());
        acc(i,2) = atof(s_split[2].c_str());
        // gyro data.
        gyro(i, 0) = atof(s_split[6].c_str());
        gyro(i, 1) = atof(s_split[7].c_str());
        gyro(i, 2) = atof(s_split[8].c_str());
        // mag data.
        mag(i,0) = atof(s_split[9].c_str()) / 1000.0;
        mag(i,1) = atof(s_split[10].c_str()) / 1000.0;
        mag(i,2) = atof(s_split[11].c_str()) / 1000.0;
        i++;
    }
    infile.close();

}


void DataFormat::split(const string &s, vector<string> &v, const string &c) {
    string::size_type pos1, pos2;
    pos2 = s.find(c);
    pos1 = 0;
    while (string::npos != pos2) {
        v.push_back(s.substr(pos1, pos2 - pos1));

        pos1 = pos2 + c.size();
        pos2 = s.find(c, pos1);
    }
    if (pos1 != s.length())
        v.push_back(s.substr(pos1));
}







