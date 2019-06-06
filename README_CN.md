# Location

> 定位是驾驶导航过程中是最基础的一步，也是十分关键的一步，一个准确的定位可以有效提高绑路的精度，也能更加精准感知驾驶形态的变化，由于本项目主要基于手机做导航定位，目前采用的是手机内置的传感器数据（陀螺仪，加速计，地磁计）以及GPS数据融合定位。

## 项目所需的传感器.

- [X] 陀螺仪(x, y, z).
- [X] 加速计(x, y, z).
- [X] 地磁计(x, y, z).
- [X] 重力感应器(x, y, z).
- [X] 方向传感器(roll, pitch, yaw).
- [X] 指南针(degree).
- [X] 道路信息(距下个路口的距离, 道路方向, 道路类型).
- [X] GPS(经度, 纬度, 海拔, 精度, 速度, 方向, 时间戳).


## 一些实现细节

- 传感器噪声过滤与修正.

<img src="https://raw.githubusercontent.com/yyccR/Pictures/master/INS/ornt_filter.png" width="1000" height="800" />

- 基于非耦合的GPS融合INS.

<img src="https://raw.githubusercontent.com/yyccR/Pictures/master/INS/origin location.png" width="430" height="350" /> <img src="https://raw.githubusercontent.com/yyccR/Pictures/master/INS/final location.png" width="430" height="350" />

## 快速开始
确保安装了gcc和cmake, 下载本项目到你的项目下

```
git clone https://github.com/yyccR/Location.git
```

在项目根目录下新建`CMakeLists.txt`, 同时添加如下:

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

在main文件里添加如下测试代码.

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

如果输出 `114.174 22.2838` 表示已经成功内嵌了本项目.

## 数据格式.

- 陀螺仪(x, y, z), 单位 rad/s

- 加速计(x, y, z), 单位 m/s²

- 地磁计(x, y, z), 单位 μt

- 重力感应器(x, y, z), 单位 m/s²

- 方向传感器(roll, pitch, yaw), 单位 角度(degree)

手机并没有方向传感器, 这个所谓的传感器数据是手机底层算法计算得到的。

- 指南针(degree), 单位 角度(degree)

- 道路信息(距离下个路口距离, 当前位置道路方向, 道路类型编码)

如果拿不到道路数据, 则全部填0即可, `(0.0, 0.0, 0.0)`

- GPS(lng, lat, alt, accuracy, speed, bearing, t)
  - lng, 经度, double
  - lat, 纬度, double
  - alt, 海拔, double
  - accuracy, 精度, double
  - speed, 速度, double
  - bearing, 方向, double, 单位 角度(degree)
  - t, 时间戳, 单位 毫秒(millisecond)

## 更加详细的调用细节

详见 docs/apiCallDetails.md

## TODO

- [X] CMakeLists 优化.
- [ ] 清理垃圾代码.
- [ ] 模板化.
- [ ] 替换普通指针为智能指针.
- [ ] 完善文档.
- [x] 添加快速开始.
- [ ] 增加更多测试案例.
- [ ] 使用合适的设计模式.


## 参考:

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