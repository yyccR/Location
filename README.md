# Location

> 定位是驾驶导航过程中是最基础的一步，也是十分关键的一步，一个准确的定位可以有效提高绑路的精度，也能更加精准感知驾驶形态的变化，由于本项目主要基于手机做导航定位，目前采用的是手机内置的传感器数据（陀螺仪，加速计，地磁计）以及GPS数据融合定位。

手机传感器数据如下：

<img src="https://raw.githubusercontent.com/yyccR/Pictures/master/Location/sensordata1.png" width="300" height="600" />



## 流程

<img src="https://raw.githubusercontent.com/yyccR/Pictures/master/INS/INS2.png" width="1000" height="400" />

## Test case

### GPS轨迹

- 融合

<img src="https://raw.githubusercontent.com/yyccR/Pictures/master/INS/GPS.png" width="370" height="170" /> <img src="https://raw.githubusercontent.com/yyccR/Pictures/master/INS/GPS2.png" width="370" height="170" />

<img src="https://raw.githubusercontent.com/yyccR/Pictures/master/INS/GPSandIMU.png" width="370" height="170" /> <img src="https://raw.githubusercontent.com/yyccR/Pictures/master/INS/GPSandIMU2.png" width="370" height="170" />

- 未融合

<img src="https://raw.githubusercontent.com/yyccR/Pictures/master/INS/gps5.png" width="370" height="170" /> <img src="https://raw.githubusercontent.com/yyccR/Pictures/master/INS/gps7.png" width="370" height="170" />

<img src="https://raw.githubusercontent.com/yyccR/Pictures/master/INS/gps6.png" width="370" height="170" /> <img src="https://raw.githubusercontent.com/yyccR/Pictures/master/INS/gps8.png" width="370" height="170" />

### IMU轨迹



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