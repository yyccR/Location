//
// Created by yangcheng on 2018/12/14.
//

#ifndef LOCATION_COORDINATE_H
#define LOCATION_COORDINATE_H

struct Point2D {
    double lng;
    double lat;

    Point2D(double _lng, double _lat) : lng(_lng), lat(_lat) {};
};

struct Point3D {
    double lng;
    double lat;
    double altitude;

    Point3D(double _lng, double _lat, double _altitude) : lng(_lng), lat(_lat), altitude(_altitude) {};
};

class Coordinate {
public:

    Coordinate();

    ~Coordinate();

    // 经纬度转墨卡托
    Point2D LngLat2Mercator(double lng, double lat);

    // 墨卡托转经纬度
    Point2D Mercator2LngLat(double x, double y);

    // 角度转弧度
    double Deg2Rad(double deg);

    // 弧度转角度
    double Rad2Deg(double rad);

};


#endif //LOCATION_COORDINATE_H
