//
// Created by yangcheng on 2019/7/10.
//

#include "TestXgboostDetector.h"
#include <fstream>
#include <iostream>
#include "../utils/Tools.h"
#include "../models/XgboostDetector.h"
//#include "utils/DataFormat.h"


TestXgboostDetector::TestXgboostDetector() {};
TestXgboostDetector::~TestXgboostDetector() {};

void TestXgboostDetector::TestDetector() {

    std::cout.precision(9);
    // read data
    std::string file_name = "C:\\Users\\yangcheng\\Desktop\\validation_data.csv";

    std::ifstream infile;
    infile.open(file_name, std::ios::in);
    long long int rows_cout = std::count(std::istreambuf_iterator<char>(infile),
                               std::istreambuf_iterator<char>(), '\n') + 1;
    infile.close();
    infile.open(file_name, std::ios::in);
    Eigen::MatrixXd data(rows_cout, 27);

    std::string s;
    int i = 0;
    Tools tools;
    while (std::getline(infile, s)) {

        std::vector<std::string> s_split = tools.split(s, ",");

        // data.
        for(int j = 0; j < 27; ++j){
            data(i, j) = stod(s_split[j]);
        }
        i +=1;
    }
    infile.close();


    // detector
    Eigen::VectorXd res(rows_cout);
    std::string model_path = "D:\\worksheet\\clion\\Location\\models\\raw_model.txt";
    XgboostDetector xgboostDetector = XgboostDetector(model_path);
    StopDetection &stopDetection = xgboostDetector;
    for(int k = 0; k < rows_cout; ++k){
        Eigen::VectorXd input = data.row(k);
        bool is_stop = stopDetection.IsStopping(input);
        res(k) = is_stop;
        std::cout << is_stop << std::endl;
    }
}