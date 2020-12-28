# 基于的matlab导航科学计算库



* VRU/AHRS姿态融合算法

* 捷联惯导速度位置姿态解算

* GPS IMU 经典ESKF融合

* UWB IMU融合

  

* 运行环境： matlab2020a 必须安装 sensor fusion toolbox

* 需要将\lib及其子目录加入matlab预设目录

| 目录                         | 说明                          |
| ---------------------------- | ----------------------------- |
| \example\ahrs_test           | AHRS测试                      |
| \example\allan_test          | allan方差                     |
| \example\calbiration_test    | 加速度计和陀螺校准测试        |
| \example\gps_kalman_test     | 纯GPS定位，采用卡尔曼滤波定位 |
| \example\ins_test            | 捷联惯导解算                  |
| \example\gps_imu_test        | 15维经典ESKF GPS+IMU组合导航  |
| \example\uwb_imu_fusion_test | UWB+IMU EKF 紧组合导航        |
| \example\uwb_test            | 纯UWB三角测距定位测试         |
| \example\basic               | 一些基础函数测试和入门例程    |



其中UWB+IMU 融合和 GPS+IMU融合就是经典的15维误差卡尔曼滤波(EKSF)，没有什么论文参考，就是N年一直用的经典的框架。见参考部分.





有问题欢迎提 git issue 或者加我微信反馈:

![](img/wechat.png)

更多内容请访问：

> * 知乎：https://www.zhihu.com/people/yang-xi-97-90
> * 网站：www.hipnuc.com



参考
> * 捷联惯导算法与组合导航原理_严恭敏
> * 多传感器融合-深蓝学院
> * https://kth.instructure.com/files/677996/download?download_frd=1
> * https://www.coursera.org/learn/state-estimation-localization-self-driving-cars

