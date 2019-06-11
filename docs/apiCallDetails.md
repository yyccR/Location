# Api call details

## Input data format

- gyroscope(x, y, z), origin gyroscope data, unit rad/s

- accelerometer(x, y, z), origin accelerometer data, unit m/s²

- geomagnetic meter(x, y, z), origin geomagnetic data, unit μt

- gravity sensor(x, y, z), origin gravity data, unit m/s²

- direction sensor(roll, pitch, yaw), origin sensor data, unit degree

Note that direction sensor doesn't exit actually , the 'sensor data' is computation result from system underlying algorithm.

- compass(degree), origin sensor data, unit degree

- road info(distance to next cross, bearing, road type)

This data is from map data, and if you couldn't search map server data, just fill in all zero `(0.0, 0.0, 0.0)`, road type now have two value, 0 represent common road, and 1 represent tunnel.

- GPS(lng, lat, alt, accuracy, speed, bearing, t)
  - lng, longitude, double
  - lat, latitude, double
  - alt, altitude, double
  - accuracy, double
  - speed, double
  - bearing, double, unit degree
  - t, timestampe, unit millisecond

## Api call details

There only one entry class needed consider, `location/Location.h`.

First need to initial the Location class.

```
#include <Eigen/Dense>
#include "location/Location.h"
using namespace routing;

Location location;
```

Second set the data frequency you provided, default is 20.

```
location.SetHz(20.0);
```

Third call the `PredictCurrentPosition` method to compute the current position according to the input sensor data.

```
location.PredictCurrentPosition(gyro_data_v, acc_data_v, mag_data_v, gps_data_v, g_data_v, ornt_data_v, road_data_v);
```

Note that all the data here using Eigen::Vector structure, for example:

```
Vector3d gyro_data_v(0.004263,0.019169,-0.001014);
Vector3d mag_data_v(-2.313675,-82.446960,-366.183838);
Vector3d acc_data_v(0.105081,0.108075,9.774973);
VectorXd gps_data_v(7);
gps_data_v << 114.174118,22.283789,0.0,0.0,24.0,0.0,1554348968704.665039;
Vector3d g_data_v(0.094139, 0.107857,9.808955);
Vector3d ornt_data_v(-0.549866,0.629957,-0.069398);
Vector3d road_data(1000.0, 0.0, 0);
```

The method `PredictCurrentPosition` will not return any values, all the process result was store in Status structure.

Final if you want to get the positioning result, just type:

```
GNSSINS gnssins = location.GetGNSSINS();
```

Below is the `GNSSINS` structure which defined in `system/Status.h`

```
struct GNSSINS {
    double lng;
    double lat;
    double altitude;
    double accuracy;
    double speed;
    double bearing;
};
```

## Input data frequency

- The frequency of `PredictCurrentPosition` calls should be the same or not very different from the frequency set by the `location.SetHz()` method.

- When GPS is in gap period, all data of GPS should be sent 0, v(0, 0, 0, 0, 0, 0, 0)

- When `road info` in gap period, the data should be the same as before, and should be set to zero during rerouting or no such server.


## Data examples

| acc_x      | acc_y     | acc_z     | g_x       | g_y       | g_z       | gyro_x       | gyro_y      | gyro_z      | mag_x   | mag_y    | mag_z    | ornt_z    | ornt_x    | ornt_y    | latitude    | longitude   | altitude | altitude-google | gps-speed   | gps-accuracy | gps-bearing | gps-Satellites | time-using | time-format | timestamp     | road-dist to next cross | road-heading | road-type |
| ---------- | --------- | --------- | --------- | --------- | --------- | ------------ | ----------- | ----------- | ------- | -------- | -------- | --------- | --------- | --------- | ----------- | ----------- | -------- | --------------- | ----------- | ------------ | ----------- | -------------- | ---------- | ----------- | ------------- | ----------------------- | ------------ | --------- |
| 1.3190615  | 7.3327217 | 6.973821  | 0.9129    | 7.2721996 | 6.5154    | 0.15892968   | 0.026389379 | 0.043231804 | 59.3125 | -25.5625 | -30.75   | 269.3907  | 47.864853 | 7.976027  | 23.14972849 | 113.3213488 | 87.41    | 0               | 0.75        | 55           | 334.0499878 | 0              | 0          | 0           | 1558586596582 | 64.29145398             | 257.9223308  | 0         |
| 0.52454084 | 7.3596897 | 5.2365403 | 0.8429    | 7.5298996 | 6.2257    | 0.29626963   | 0.07393215  | 0.06407104  | 59.75   | -24.9375 | -28.25   | 269.42166 | 50.160385 | 7.7104144 | 0           | 0           | 0        | 0               | 0           | 0            | 0           | 0              | 0          | 0           | 0             | 64.29145398             | 257.9223308  | 0         |
| 1.1784357  | 7.0913377 | 4.977162  | 0.8384    | 7.5929    | 6.1492996 | 0.162176     | 0.15879005  | 0.031991884 | 59.9375 | -24.875  | -28.25   | 269.6926  | 50.73855  | 7.763878  | 0           | 0           | 0        | 0               | 0           | 0            | 0           | 0              | 0          | 0           | 0             | 64.29145398             | 257.9223308  | 0         |
| 0.2054225  | 6.91365   | 5.1467767 | 0.7237    | 7.6397996 | 6.1056    | -0.014154621 | 0.6226113   | 0.15730652  | 60.25   | -25      | -26.6875 | 269.49725 | 51.173664 | 6.75976   | 0           | 0           | 0        | 0               | 0           | 0            | 0           | 0              | 0          | 0           | 0             | 64.29145398             | 257.9223308  | 0         |
| 0.6924606  | 7.58598   | 6.012683  | 0.6529    | 7.634     | 6.1208    | -0.11070623  | 0.2043955   | 0.10725048  | 60.3125 | -25.4375 | -26.0625 | 269.29425 | 51.119728 | 6.0886636 | 0           | 0           | 0        | 0               | 0           | 0            | 0           | 0              | 0          | 0           | 0             | 64.29145398             | 257.9223308  | 0         |
| 1.6083577  | 7.45473   | 7.067147  | 0.6153    | 7.5783997 | 6.1934    | -0.19556414  | 0.43769366  | 0.14540339  | 60.9375 | -26.5    | -24.75   | 267.7838  | 50.604874 | 5.6735864 | 0           | 0           | 0        | 0               | 0           | 0            | 0           | 0              | 0          | 0           | 0             | 64.29145398             | 257.9223308  | 0         |
| 0.23061907 | 6.588575  | 6.834823  | 0.5119    | 7.5508    | 6.2363997 | 0.22787018   | 0.29204595  | 0.06716027  | 61      | -27.3125 | -23.5    | 267.04623 | 50.351337 | 4.6924677 | 0           | 0           | 0        | 0               | 0           | 0            | 0           | 0              | 0          | 0           | 0             | 64.29145398             | 257.9223308  | 0         |
| 1.0430675  | 7.6366796 | 5.488736  | 0.4615    | 7.6019998 | 6.1777997 | 0.16502088   | 0.5134933   | 0.259513    | 61.125  | -27.9375 | -22.4375 | 266.4972  | 50.822758 | 4.2722297 | 0           | 0           | 0        | 0               | 0           | 0            | 0           | 0              | 0          | 0           | 0             | 64.29145398             | 257.9223308  | 0         |
| 2.6479583  | 6.8948793 | 6.831893  | 1.9764999 | 6.5432    | 7.0319996 | 0.081279986  | 0.031311207 | -0.07298967 | 53.9375 | -53.375  | -18      | 236.26198 | 41.853275 | 15.6992   | 23.14982671 | 113.3206776 | 131.78   | 0               | 2.700000048 | 47           | 258.3500061 | 0              | 0          | 0           | 1558586640000 | 47.3517091              | 257.9223308  | 0         |
| 1.9768157  | 6.7945433 | 6.9858017 | 1.9095    | 6.5825    | 7.0137997 | 0.059236474  | 0.13137093  | -0.16161749 | 54.375  | -53.125  | -17.6875 | 236.70316 | 42.16233  | 15.22961  | 0           | 0           | 0        | 0               | 0           | 0            | 0           | 0              | 0          | 0           | 0             | 47.3517091              | 257.9223308  | 0         |


- Note that in gps gap time, all the gps data should be fill zero, and the road info should be the same as before.