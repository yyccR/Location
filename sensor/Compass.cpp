//
// Created by yangcheng on 2019/4/17.
//

#include "Compass.h"

using namespace Eigen;

bool Compass::IsCompassVaild(routing::Status *status, Eigen::Vector3d &ornt_data) {

    static VectorXd sin_compass_queue((*status).parameters.compass_queue_len);
    static VectorXd cos_compass_queue((*status).parameters.compass_queue_len);
    static int cnt = 0;

    if (cnt < (*status).parameters.compass_queue_len) {
        sin_compass_queue(cnt) = sin(ornt_data(2) * M_PI / 180.0);
        cos_compass_queue(cnt) = cos(ornt_data(2) * M_PI / 180.0);
        cnt += 1;
        return true;
    } else {
        double sin_mean = sin_compass_queue.mean();
        double cos_mean = cos_compass_queue.mean();
        double sin_var = (sin_compass_queue.array() - sin_mean).pow(2).sum() / (*status).parameters.compass_queue_len;
        double cos_var = (cos_compass_queue.array() - cos_mean).pow(2).sum() / (*status).parameters.compass_queue_len;
//        std::cout << ornt_data(2) << " " << sin_var << " " << cos_var << std::endl;
        for (int i = 0; i < (*status).parameters.compass_queue_len - 1; i++) {
            sin_compass_queue(i) = sin_compass_queue(i + 1);
            cos_compass_queue(i) = cos_compass_queue(i + 1);
        }
        sin_compass_queue((*status).parameters.compass_queue_len - 1) = sin(ornt_data(2) * M_PI / 180.0);
        cos_compass_queue((*status).parameters.compass_queue_len - 1) = cos(ornt_data(2) * M_PI / 180.0);
        return sin_var < (*status).parameters.compass_vaild_var_thres && cos_var < (*status).parameters.compass_vaild_var_thres;
    }

}