//
// Created by yangcheng on 2019/1/17.
//

#include "DataFormat.h"
#include "fstream"
#include "cassert"
#include "iostream"
#include <cmath>


using namespace std;
using namespace Eigen;

void DataFormat::readCSV(MatrixXd &gyro, MatrixXd &acc, MatrixXd &mag, MatrixXd &gps_data, MatrixXd &g_data, MatrixXd &ornt_data, MatrixXd &road_data) {
    ifstream infile;
    // D:\worksheet\clion\Location\test\data\EEWalk2.csv  McLaneWalk.csv
    // D:\worksheet\clion\Location\test\data\sensor_log\origin_sensors_data_1553783441847.0.csv
    // D:\worksheet\clion\Location\test\data\sensor_log\origin_sensors_data_1553783672990.0.csv
    // D:\worksheet\clion\Location\test\data\sensor_log\origin_sensors_data_1554081875000.0.csv
    // D:\worksheet\clion\Location\test\data\sensor_log\origin_sensors_data_1554081886000.0.csv
    // D:\worksheet\clion\Location\test\data\sensor_log\origin_sensors_data_1554773520000.0.csv
    // D:\worksheet\clion\Location\test\data\sensor_log\origin_sensors_data_1554989169000.0.csv
    // D:\worksheet\clion\Location\test\data\sensor_log\origin_sensors_data_1554988196000.0.csv
    // D:\worksheet\clion\Location\test\data\sensor_log\origin_sensors_data_1555072316000.0.csv
    // D:\worksheet\clion\Location\test\data\sensor_log\origin_sensors_data_1555401659314.0.csv
    // D:\worksheet\clion\Location\test\data\sensor_log\origin_sensors_data_1555461064000.0.csv
    // D:\worksheet\clion\Location\test\data\sensor_log\origin_sensors_data_1555506970000.0.csv
    // D:\worksheet\clion\Location\test\data\sensor_log\origin_sensors_data_1555592383547.0.csv
    // D:\worksheet\clion\Location\test\data\sensor_log\origin_sensors_data_1556024437000.0.csv
    // D:\worksheet\clion\Location\test\data\sensor_log\origin_sensors_data_1556026699999.0.csv
    // D:\worksheet\clion\Location\test\data\sensor_log\origin_sensors_data_1556110776000.0.csv
    // D:\worksheet\clion\Location\test\data\sensor_log\origin_sensors_data_1556115336000.0.csv
    // D:\worksheet\clion\Location\test\data\sensor_log\origin_sensors_data_1557061333000.0.csv
    // D:\worksheet\clion\Location\test\data\sensor_log\origin_sensors_data_1557127554996.0.csv
    // D:\worksheet\clion\Location\test\data\sensor_log\origin_sensors_data_1557128036000.0.csv
    infile.open("D:\\worksheet\\clion\\Location\\test\\data\\sensor_log\\origin_sensors_data_1557128036000.0.csv",
                ios::in);
    assert(infile.is_open());


//    MatrixXd v;
    string s;
    int i = 0;
    while (getline(infile, s)) {

        vector<string> s_split;
        split(s, s_split, ",");

        // acc data.
        acc(i, 0) = atof(s_split[0].c_str());
        acc(i, 1) = atof(s_split[1].c_str());
        acc(i, 2) = atof(s_split[2].c_str());
        // gravity data.
        g_data(i,0) = atof(s_split[3].c_str());
        g_data(i,1) = atof(s_split[4].c_str());
        g_data(i,2) = atof(s_split[5].c_str());
        // gyro data.
        gyro(i, 0) = atof(s_split[6].c_str());
        gyro(i, 1) = atof(s_split[7].c_str());
        gyro(i, 2) = atof(s_split[8].c_str());
//        gyro(i, 0) = atof(s_split[13].c_str());
//        gyro(i, 1) = atof(s_split[14].c_str());
//        gyro(i, 2) = atof(s_split[12].c_str());
        // mag data.
        mag(i, 0) = atof(s_split[9].c_str());
        mag(i, 1) = atof(s_split[10].c_str());
        mag(i, 2) = atof(s_split[11].c_str());
        // gps data, gps(lng,lat,alt,accuracy,speed,bearing,t)
        double lng = atof(s_split[16].c_str());
        double lat = atof(s_split[15].c_str());
//        if (i != 0) {
        if (i != 0 && lat == gps_data(i - 1, 1) && lng == gps_data(i - 1, 0)) {
            gps_data(i, 0) = lng;
            gps_data(i, 1) = lat;
            gps_data(i, 2) = atof(s_split[17].c_str());
            gps_data(i, 3) = atof(s_split[20].c_str());
//            gps_data(i, 3) = 200.0;
            gps_data(i, 4) = atof(s_split[19].c_str());
//            gps_data(i, 4) = atof(s_split[19].c_str()) * 1000.0 / 3600.0;
            gps_data(i, 5) = atof(s_split[21].c_str());
            gps_data(i, 6) = atof(s_split[25].c_str());
        } else {
            gps_data(i, 0) = lng;
            gps_data(i, 1) = lat;
            gps_data(i, 2) = atof(s_split[17].c_str());
            gps_data(i, 3) = atof(s_split[20].c_str());;
//            gps_data(i, 3) = 10.0;
            gps_data(i, 4) = atof(s_split[19].c_str());
//            gps_data(i, 4) = atof(s_split[19].c_str()) * 1000.0 / 3600.0;
            gps_data(i, 5) = atof(s_split[21].c_str());
            gps_data(i, 6) = atof(s_split[25].c_str());
        }

//        ornt_data(i, 2) = atof(s_split[12].c_str()) / M_PI * 180.0;
//        ornt_data(i, 0) = atof(s_split[13].c_str()) / M_PI * 180.0;
//        ornt_data(i, 1) = atof(s_split[14].c_str()) / M_PI * 180.0;

        ornt_data(i, 2) = atof(s_split[12].c_str());
        ornt_data(i, 0) = atof(s_split[13].c_str());
        ornt_data(i, 1) = atof(s_split[14].c_str());

//        heading(i) = atof(s_split[12].c_str());

        road_data(i, 0) = atof(s_split[27].c_str());
        road_data(i, 1) = atof(s_split[26].c_str());
        road_data(i, 2) = atof(s_split[28].c_str());


        i++;
    }
//    std::cout << gps_data.rows() << std::endl;

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

void DataFormat::readCaliData(MatrixXd &gyro, MatrixXd &acc, MatrixXd &mag) {

    string a[6];
    a[0] = "D:/worksheet/clion/Location/test/data/Sensor_record_20151030_110329_AndroSensor.csv";
    a[1] = "D:/worksheet/clion/Location/test/data/Sensor_record_20151030_105902_AndroSensor.csv";
    a[2] = "D:/worksheet/clion/Location/test/data/Sensor_record_20151030_110417_AndroSensor.csv";
    a[3] = "D:/worksheet/clion/Location/test/data/Sensor_record_20151030_110448_AndroSensor.csv";
    a[4] = "D:/worksheet/clion/Location/test/data/Sensor_record_20151030_110521_AndroSensor.csv";
    a[5] = "D:/worksheet/clion/Location/test/data/Sensor_record_20151030_110553_AndroSensor.csv";


    int i = 0;
//    double temp = 0;
    for (const auto &j : a) {

        ifstream infile;
        infile.open(j, ios::in);
        assert(infile.is_open());
        string s;

        while (getline(infile, s)) {

            vector<string> s_split;
            split(s, s_split, ",");

            // acc data.
            acc(i, 0) = atof(s_split[0].c_str());
            acc(i, 1) = atof(s_split[1].c_str());
            acc(i, 2) = atof(s_split[2].c_str());
            // gyro data.
            gyro(i, 0) = atof(s_split[6].c_str());
            gyro(i, 1) = atof(s_split[7].c_str());
            gyro(i, 2) = atof(s_split[8].c_str());
            // mag data.
            mag(i, 0) = atof(s_split[9].c_str()) / 1000.0;
            mag(i, 1) = atof(s_split[10].c_str()) / 1000.0;
            mag(i, 2) = atof(s_split[11].c_str()) / 1000.0;
//            double n = mag.row(i).norm();
//            temp += n;
//            cout << n << endl;
            i++;
        }
        infile.close();
    }
//    cout << temp / i << endl;
}





