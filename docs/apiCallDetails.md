# Api call details.

## Input data format

- gyroscope(x, y, z), origin gyroscope data, unit rad/s

- accelerometer(x, y, z), origin accelerometer data, unit m/s²

- geomagnetic meter(x, y, z), origin geomagnetic data, unit μt

- gravity sensor(x, y, z), origin gravity data, unit m/s²

- direction sensor(roll, pitch, yaw), origin sensor data, unit degree

Note that direction sensor doesn't exit actually , the 'sensor data' is computation result from system underlying algorithm.

- compass(degree), origin sensor data, unit degree

- road info(distance to next cross, bearing, road type)

This data is from map data, and if you couldn't search map server data, just fill in all zero `(0.0, 0.0, 0.0)`

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
