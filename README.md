# Location

> Positioning is the most basic and crucial step in the driving navigation. An accurate positioning can effectively improve the accuracy of the road-binding, and can also sense the change of the driving pattern more accurately. Since the project is mainly based on mobile phones for navigation and positioning, Currently used is the built-in sensor data (gyroscope, accelerometer, geomagnetic meter, direction sensor, gravity sensor) and GPS data fusion positioning.

## Smartphone sensor data：

<img src="https://raw.githubusercontent.com/yyccR/Pictures/master/Location/sensordata1.png" width="300" height="600" />



## Workflow:

<img src="https://raw.githubusercontent.com/yyccR/Pictures/master/INS/framework.png" width="1100" height="800" />

## Sensors data correction

- orientation data filter(Using IIR low pass filter), blue line is the origin data, orange lie is the filter data

<img src="https://raw.githubusercontent.com/yyccR/Pictures/master/INS/ornt_filter.png" width="1000" height="800" />

- Because the posture of the smartphone could be arbitrary, so we need the gps bearing and road heading to correct the compass, below show some difference between compass and gps bearing.

<img src="https://raw.githubusercontent.com/yyccR/Pictures/master/INS/gps_compass.png" width="1000" height="300" />

## GPS trajectory and INS trajectory.

<img src="https://raw.githubusercontent.com/yyccR/Pictures/master/INS/origin location.png" width="430" height="350" /> <img src="https://raw.githubusercontent.com/yyccR/Pictures/master/INS/final location.png" width="430" height="350" />

### Real road test

- During the movement, the road is not tied, and GPS is restored after GPS is shielded

<img src="https://raw.githubusercontent.com/yyccR/Pictures/master/INS/ins_gps_test_case1.gif" width="300" height="600" />

## TODO

- [X] improve CMake.
- [ ] Clean the garbage code.
- [ ] Template processing.
- [ ] Using smart pointer instead.
- [ ] Complete all kinds of documents.
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