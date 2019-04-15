//
// Created by yangcheng on 2019/4/13.
//

#include "Gravity.h"
#include <iostream>
#include <queue>

using namespace Eigen;

bool Gravity::IsShaking(routing::Status *status, Eigen::Vector3d &g_data) {

    static std::queue<Vector3d> pre_g_data;
    static int cnt = 0;


    if(cnt <= 20.0){
        pre_g_data.push(g_data);
        cnt += 1;
//        std::cout << g_data(0) << " " << g_data(1) << " " << g_data(2) << " " << 0 << " " << 0 << std::endl;
        return false;
    }else{
        double diff_pre_curent = (g_data - pre_g_data.front()).squaredNorm();
        pre_g_data.pop();
        pre_g_data.push(g_data);
//        std::cout << g_data(0) << " " << g_data(1) << " " << g_data(2) << " " << diff_pre_curent << " " << (diff_pre_curent > 0.5) << std::endl;
        return diff_pre_curent > 0.5;
    }

//    cnt += 1;
//    std::cout.precision(9);
//    if(cnt % 20 == 0.0){
//        double diff_pre_curent = (g_data - pre_g_data).squaredNorm();
//        pre_g_data = g_data;
//        std::cout << g_data(0) << " " << g_data(1) << " " << g_data(2) << " " << diff_pre_curent << " " << (diff_pre_curent > 0.1) << std::endl;
//        return diff_pre_curent > 0.1;
//    }else{
//        std::cout << g_data(0) << " " << g_data(1) << " " << g_data(2) << " " << 0 << " " << 0 << std::endl;
//        return false;
//    }

}