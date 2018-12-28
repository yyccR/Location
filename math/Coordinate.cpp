//
// Created by yangcheng on 2018/12/14.
//

#include "Coordinate.h"
#include "math.h"

using namespace std;

Coordinate::Coordinate() {}

Coordinate::~Coordinate() = default;

Point2D Coordinate::LngLat2Mercator(double lng, double lat) {
    double x = lng * 20037508.34 / 180.0;
    double y = std::log(std::tan((90.0 + lat) * M_PI / 360.0)) / (M_PI / 180.0);
    y *= 20037508.34 / 180.0;
    return {x, y};
}

Point2D Coordinate::Mercator2LngLat(double x, double y) {
    double lng = x / 20037508.34 * 180.0;
    double lat = y / 20037508.34 * 180.0;
    lat = 180.0 / M_PI * (2 * std::atan(std::exp(lat * M_PI / 180.0)) -  M_PI / 2.0);
    return {lng, lat};
}

double Coordinate::Deg2Rad(double deg) {
    return deg * M_PI / 180.0;
}

double Coordinate::Rad2Deg(double rad) {
    return rad * 180.0 / M_PI;
}