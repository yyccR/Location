# Location

> 定位是驾驶导航过程中是最基础的一步，也是十分关键的一步，一个准确的定位可以有效提高绑路的精度，也能更加精准感知驾驶形态的变化，由于本项目主要基于手机做导航定位，目前采用的是手机内置的传感器数据（陀螺仪，加速计，地磁计）以及GPS数据融合定位。

手机传感器数据如下：

<img src="https://raw.githubusercontent.com/yyccR/Pictures/master/Location/sensordata1.png" width="300" height="600" />



## Test case


### 测试优化器LM算法，高斯牛顿算法
```
 标准化后的数据:
  0.0438776   0.0326531     1.00102
  0.0214286   0.0255102     0.99898
 -0.0132653   0.0122449     1.00204
-0.00102041   -0.022449    0.996939
   0.044898  -0.0214286      1.0051
  0.0520408  -0.0346939     1.00816

 计算f(x):
-0.00503332
0.000929821
 -0.0044117
 0.00560808
 -0.0127051
 -0.0203051

 计算jacobi:
   0.0877551    0.0653061      2.00204  -0.00385048  -0.00213244     -2.00408
   0.0428571    0.0510204      1.99796 -0.000918367  -0.00130154     -1.99592
  -0.0265306    0.0244898      2.00408 -0.000351937 -0.000299875     -2.00817
 -0.00204082    -0.044898      1.99388 -2.08247e-06  -0.00100791     -1.98777
   0.0897959   -0.0428571       2.0102  -0.00403165 -0.000918367     -2.02046
    0.104082   -0.0693878      2.01633  -0.00541649  -0.00240733     -2.03279

 初始coef:
0
0
0
1
1
1

 优化后coef:
 0.00401242
-0.00272034
 0.00102901
   0.999798
   0.999966
   0.998107
```


## reference:

1. [An efficient orientation filter for inertial and
    inertial/magnetic sensor arrays](http://x-io.co.uk/res/doc/madgwick_internal_report.pdf)
2. [Estimation of IMU and MARG orientation using a gradient descent algorithm](http://vigir.missouri.edu/~gdesouza/Research/Conference_CDs/RehabWeekZ%C3%BCrich/icorr/papers/Madgwick_Estimation%20of%20IMU%20and%20MARG%20orientation%20using%20a%20gradient%20descent%20algorithm_ICORR2011.pdf)
3. [Direction Cosine Matrix IMU Theory](https://www.researchgate.net/publication/265755808_Direction_Cosine_Matrix_IMU_Theory)
4. 《惯性导航》秦永元
5. [METHODS FOR NON-LINEAR LEAST SQUARES PROBLEMS](http://www2.imm.dtu.dk/pubdb/views/edoc_download.php/3215/pdf/imm3215.pdf)
6.