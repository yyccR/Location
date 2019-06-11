# Sensor data checking

- [X] checking orientation sensor data

The orientation sensor contains roll, pitch and yaw three values. Actually it is not a hardware output, it is
calculated by accelerometer and magnetometer. Some smartphone systems may contain the method
`getRotationMatrix` and `getOrientation` to calculate the orientation, so if we have got the orientation data,
we need to make sure it's available, below show the steps how to do these:

Suppose we have got the orientation `(57.221,-0.543, 143.2)` and the corresponding gravity
`(0.041, 8.248, 5.311)`, then using the follow code:

```
#include <iostream>
#include <Eigen/Dense>
#include <math/Quaternions.cpp>
using namespace std;

Eigen::Vector3d e(57.221,-0.543, 143.2);
Quaternions quaternions;
Eigen::Vector4d q = quaternions.GetQFromEuler(e);
Eigen::MatrixXd dcm = quaternions.GetDCMFromQ(q);
Eigen::Vector3d gb(0.041, 8.248, 5.311);
Eigen::Vector3d gn = dcm * gb;
cout << gn.transpose() << endl;
```
  The code above doing three things:
  1. convert the orientation into quaternion.
  2. convert the quaternion into direction cosine matrix(DCM).
  3. using the DCM to rotate the gravity.

  We can see the output vector is `(0.0415164,-0.0312637,9.80995)`, that means we succeed in
rotating the gravity from body frame to navigation frame using the orientation data.

  Because the gravity in navigation frame should always similar to  `(0,0,g)`, so we can say that the
orientation data is available if it can rotate the origin gravity data into `(0,0,g)`.


- [ ] checking magnetometer sensor data

- [ ] checking accelerometer sensor data

- [ ] checking gyroscope sensor data
