//
// Created by yangcheng on 2019/1/17.
//

#include "DataFormat.h"
#include "fstream"
#include "cassert"
#include "../../math/Quaternions.h"
#include "../../sensor/Accelerometer.h"
#include "iostream"
#include <cmath>
#include <dirent.h>

//using namespace std;
using namespace Eigen;

std::vector<std::string> DataFormat::getAllfiles(std::string &dir) {
    DIR *dir_buffer;
    struct dirent *ent;
    std::vector<std::string> files;
    if ((dir_buffer = opendir(dir.c_str())) != NULL) {
        while ((ent = readdir(dir_buffer)) != NULL) {
            string file_name(ent->d_name);
            if (file_name != "." && file_name != ".."){
                files.push_back(file_name);
            }
        }
        closedir(dir_buffer);
    } else {
        /* could not open directory */
        std::cout << "Director Open ERROR." << std::endl;
    }
    return files;
}

void DataFormat::writeCSVs() {

    std::string in_dir = "D:\\worksheet\\pyCharm\\work sheet\\AccSpeedFitting\\data\\sensor_data\\";
    std::string out_dir = "D:\\worksheet\\pyCharm\\work sheet\\AccSpeedFitting\\data\\sensor_data_format\\";

    std::vector<std::string> files = getAllfiles(in_dir);

    for(auto &file : files){
        std::ifstream inFile(in_dir + file);
        int file_cout = std::count(std::istreambuf_iterator<char>(inFile),
                           std::istreambuf_iterator<char>(), '\n') + 1;
        inFile.close();

        MatrixXd gyro_data(file_cout,3),acc_data(file_cout,3),mag_data(file_cout,3), gps_data(file_cout,7),
                g_data(file_cout,3), ornt_data(file_cout,3), road_data(file_cout,3);

        readCSV(in_dir+file, gyro_data, acc_data, mag_data, gps_data, g_data, ornt_data, road_data);

        writeCSV(out_dir+file,acc_data, g_data, gyro_data, mag_data, ornt_data, gps_data);
    }

}

void DataFormat::writeCSV(std::string file_name, MatrixXd &acc, MatrixXd &g_data, MatrixXd &gyro,
                          MatrixXd &mag, MatrixXd &ornt_data, MatrixXd &gps_data) {

    ofstream outfile;
    outfile.open(file_name, ios::trunc);

    int data_size = acc.rows();
    Quaternions quaternions;
    Accelerometer accelerometer;
    outfile.precision(std::numeric_limits<double>::digits10 + 1);
    for(int i = 1; i < data_size; ++i){

        Vector3d pre_ornt = ornt_data.row(i-1);
        Vector3d current_ornt = ornt_data.row(i);
        Vector3d ornt_diff = current_ornt - pre_ornt;
        Vector4d q = quaternions.GetQFromEuler(current_ornt);
        Matrix3d dcm = quaternions.GetDCMFromQ(q);

        Vector3d acc_v = acc.row(i);
        Vector3d acc_n = dcm * acc_v;
        Vector3d acc_v_norm = accelerometer.Normalise(acc_v);
        Vector3d acc_n_norm = accelerometer.Normalise(acc_n);

        Vector3d g_v = g_data.row(i);
        Vector3d g_n = dcm * g_v;
        Vector3d g_v_norm = accelerometer.Normalise(g_v);
        Vector3d g_n_norm = accelerometer.Normalise(g_n);
        Vector3d a_diff = acc_n - g_n;

        Vector3d gyro_v = gyro.row(i);
        Vector3d gyro_v_norm = accelerometer.Normalise(gyro_v);

        Vector3d pre_mag_v = mag.row(i-1);
        Vector3d current_mag_v = mag.row(i);
        Vector3d mag_v_diff = (dcm * current_mag_v) - (dcm * pre_mag_v);
        Vector3d mag_v_norm = accelerometer.Normalise(mag_v_diff);

        VectorXd gps_v = gps_data.row(i);

        outfile << acc_v_norm(0) << "," << acc_v_norm(1) << "," << acc_v_norm(2) << ",";
        outfile << acc_n_norm(0) << "," << acc_n_norm(1) << "," << acc_n_norm(2) << ",";
        outfile << g_v_norm(0) << "," << g_v_norm(1) << "," << g_v_norm(2) << ",";
        outfile << g_n_norm(0) << "," << g_n_norm(1) << "," << g_n_norm(2) << ",";
        outfile << gyro_v_norm(0) << "," << gyro_v_norm(1) << "," << gyro_v_norm(2) << ",";
        outfile << mag_v_norm(0) << "," << mag_v_norm(1) << "," << mag_v_norm(2) << ",";
        outfile << mag_v_diff(0) << "," << mag_v_diff(1) << "," << mag_v_diff(2) << ",";
        outfile << a_diff(0) << "," << a_diff(1) << "," << a_diff(2) << ",";
        outfile << ornt_diff(0) << "," << ornt_diff(1) << "," << ornt_diff(2) << ",";
        outfile << gps_v(0) << "," << gps_v(1) << "," << gps_v(2) << ","
                << gps_v(3) << "," << gps_v(4) << "," << gps_v(5) << ","
                << gps_v(6) << endl;
    }
    std::cout << "write file: " << file_name <<  "data size is " << data_size << std::endl;
    outfile.close();


}


void DataFormat::readCSV(std::string file_name, MatrixXd &gyro, MatrixXd &acc, MatrixXd &mag,
                         MatrixXd &gps_data, MatrixXd &g_data, MatrixXd &ornt_data, MatrixXd &road_data) {
    ifstream infile;
    cout << "read file: " << file_name << endl;
    infile.open(file_name,
                ios::in);
    assert(infile.is_open());

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

        road_data(i, 0) = atof(s_split[26].c_str());
        road_data(i, 1) = atof(s_split[27].c_str());
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





