# LOCATION

[![Build Status](https://travis-ci.org/yyccR/Location.svg?branch=master)](https://travis-ci.org/yyccR/Location)

> Positioning is the most basic and crucial step in the driving navigation. An accurate positioning can effectively improve the accuracy of the road-binding, and can also sense the change of the driving pattern more accurately. Since the project is mainly based on mobile phones for navigation and positioning, Currently used is the built-in sensor data (gyroscope, accelerometer, geomagnetic meter, direction sensor, gravity sensor) and GPS data fusion positioning.

## Sensor data required.

- [X] gyroscope(x, y, z).
- [X] accelerometer(x, y, z).
- [X] geomagnetic meter(x, y, z).
- [X] gravity sensor(x, y, z).
- [X] direction sensor(roll, pitch, yaw).
- [X] compass(degree).
- [X] road info(distance to next cross, bearing, road type).
- [X] GPS(lng, lat, alt, accuracy, speed, bearing, t).


## Some implement details

- sensor data filter.

<img src="https://raw.githubusercontent.com/yyccR/Pictures/master/INS/ornt_filter.png" width="1000" height="800" />

- GPS fusion INS under uncoupling system.

<img src="https://raw.githubusercontent.com/yyccR/Pictures/master/INS/origin location.png" width="430" height="350" /> <img src="https://raw.githubusercontent.com/yyccR/Pictures/master/INS/final location.png" width="430" height="350" />

## Quick start
First make sure gcc and cmake was installed, and include this library into your project.

```
git clone https://github.com/yyccR/Location.git
```

Second open your `CMakeLists.txt` and add these:

```
include_directories(${PROJECT_SOURCE_DIR}/Location/include/eigen3)

include_directories(${PROJECT_SOURCE_DIR}/Location/math)
add_subdirectory(Location/math)

include_directories(${PROJECT_SOURCE_DIR}/Location/models)
add_subdirectory(Location/models)

include_directories(${PROJECT_SOURCE_DIR}/Location/location)
add_subdirectory(Location/location)

include_directories(${PROJECT_SOURCE_DIR}/Location/sensor)
add_subdirectory(Location/sensor)

include_directories(${PROJECT_SOURCE_DIR}/Location/system)
add_subdirectory(Location/system)

target_link_libraries(${PROJECT_NAME} Location_math)
target_link_libraries(${PROJECT_NAME} Location_models)
target_link_libraries(${PROJECT_NAME} Location_location)
target_link_libraries(${PROJECT_NAME} Location_sensor)
target_link_libraries(${PROJECT_NAME} Location_system)
target_link_libraries(${PROJECT_NAME} Location_test)
```

final open your main file, and add the test code.

```
#include <iomanip>
#include <Eigen/Dense>
#include "sensor/GPS.h"
#include "location/Location.h"

using namespace Eigen;
using namespace std;

int main() {

    Location location;
    Vector3d gyro_data_v(0.004263,0.019169,-0.001014);
    Vector3d mag_data_v(-2.313675,-82.446960,-366.183838);
    Vector3d acc_data_v(0.105081,0.108075,9.774973);
    VectorXd gps_data_v(7);
    gps_data_v << 114.174118,22.283789,0.0,0.0,24.0,0.0,1554348968704.665039;
    Vector3d g_data_v(0.094139, 0.107857,9.808955);
    Vector3d ornt_data_v(-0.549866,0.629957,-0.069398);
    Vector3d road_data(1000.0, 0.0, 0);
    location.PredictCurrentPosition(gyro_data_v,acc_data_v,mag_data_v,gps_data_v,g_data_v,ornt_data_v, road_data);
    cout << location.GetGNSSINS().lng << " " << location.GetGNSSINS().lat << endl;
    return 0;
}
```

if you see the output `114.174 22.2838` that means this library was embedded to your project successfully.

## Input data format.

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

## More detail tutorial.

- [Api calls details](docs/apiCallDetails.md)
- [Sensor data checking](docs/SensorDataChecking.md)
- [Impelement details](docs/implementDetails.md)
- [Sensor calibration](docs/SensorCalibration.md)
- [Training Stop detection model](docs/trainingStopDetectModel.md)

## TODO

- [X] improve CMake.
- [X] Clean the garbage code.
- [ ] Template processing.
- [X] Using smart pointer instead.
- [X] Complete all kinds of documents.
- [X] Add quick start.
- [ ] Add more test case.
- [ ] Design a suitable pattern.

## reference:

1. 《惯性导航》秦永元
2. 《捷联惯性导航技术(第2版 译本)》译者:张天光/王秀萍/王丽霞 作者:DavidH.Titte
3. [An efficient orientation filter for inertial and
    inertial/magnetic sensor arrays](http://x-io.co.uk/res/doc/madgwick_internal_report.pdf)
4. [Estimation of IMU and MARG orientation using a gradient descent algorithm](http://vigir.missouri.edu/~gdesouza/Research/Conference_CDs/RehabWeekZ%C3%BCrich/icorr/papers/Madgwick_Estimation%20of%20IMU%20and%20MARG%20orientation%20using%20a%20gradient%20descent%20algorithm_ICORR2011.pdf)
5. [Direction Cosine Matrix IMU Theory](https://www.researchgate.net/publication/265755808_Direction_Cosine_Matrix_IMU_Theory)
6. [METHODS FOR NON-LINEAR LEAST SQUARES PROBLEMS](http://www2.imm.dtu.dk/pubdb/views/edoc_download.php/3215/pdf/imm3215.pdf)
7. [A Calibration Algorithm for Microelectromechanical Systems Accelerometers in Inertial Navigation Sensors](https://arxiv.org/pdf/1309.5075.pdf)
8. [A Calibration Method of Three-axis Magnetic Sensor Based on Ellipsoid Fitting](https://www.researchgate.net/publication/273845104_A_Calibration_Method_of_Three-axis_Magnetic_Sensor_Based_on_Ellipsoid_Fitting)
9. [Accuracy Improvement of Low Cost INS/GPS for Land Applications](https://prism.ucalgary.ca/bitstream/handle/1880/41142/2001_Shin.pdf?sequence=1)
10. [Trajectory preprocessing: Computing with Spatial Trajectories](https://books.google.com.hk/books?hl=zh-CN&lr=&id=JShQJF23xBgC&oi=fnd&pg=PR3&dq=Trajectory+preprocessing.+Computing+with+Spatial+Trajectories&ots=6NUeew5i9_&sig=o7XM_QcuUnmOv5KNeezTN4H8PMw&redir_esc=y&hl=zh-CN&sourceid=cndr#v=onepage&q=Trajectory%20preprocessing.%20Computing%20with%20Spatial%20Trajectories&f=false)