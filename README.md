# 基于MATLAB的导航科学计算库

教学性质的：

* GPS IMU经典15维ESKF松组合
* VRU/AHRS姿态融合算法
* 捷联惯导速度位置姿态解算例子
* UWB IMU紧组合融合
* 每个例子自带数据集

运行环境：

* MATLAB R2020a必须安装sensor fusion toolbox和navigation tool box

* 需要将`\lib`及其子目录加入MATLAB预设目录， 或者运行一下根目录下的`init.m`

| 目录                         | 说明                                                         |
| ---------------------------- | ------------------------------------------------------------ |
| \example\vru_ahrs_test       | AHRS/IMU测试                                                 |
| \example\allan_test          | Allan方差                                                    |
| \example\calibration_test    | 加速度计和陀螺校准测试                                       |
| \example\gps_kalman_test     | 纯GPS定位，采用卡尔曼滤波定位                                |
| \example\ins_test            | 捷联惯导解算                                                 |
| \example\gps_imu_test        | 15维ESKF GPS+IMU组合导航                                     |
| \example\uwb_imu_fusion_test | 15维UWB+IMU EKF紧组合                                        |
| \example\uwb_test            | 纯UWB多边定位测试                                            |
| \example\basic               | 一些基础函数测试和入门例程(入门推荐先看)                     |
| \example\其他未列出目录      | 试验性质的测试代码，建议不看                                 |
| \study                       | 自己学习用脚本，包含线性代数/和一些参考书籍书后MATLAB答案及验证，一般不需要，代码不保证能运行，一般不需要请删除 |
| \lib                         | 存放库文件或者经过验证过的公共函数，可靠性较高               |

其中UWB+IMU融合和GPS+IMU融合就是经典的15维误差卡尔曼滤波(EKSF)，没有什么论文参考，就是一直用的经典的框架(就是松组合)，见参考部分。

有问题欢迎提git issue或者加QQ群讨论：138899875

![](img/wechat.png)

更多内容请访问：

- 知乎：https://www.zhihu.com/people/yang-xi-97-90
- 网站：www.hipnuc.com

参考：

1. 书：捷联惯导算法与组合导航原理 严恭敏及PSINS工具箱官方网站：http://www.psins.org.cn/sy
2. 书：GNSS与惯性及多传感器组合导航系统原理 第二版
3. 书：GPS原理与接收机设计 谢刚
4. 深蓝学院-多传感器融合课程(理论推导及code)
5. 武汉大学牛小骥惯性导航课程(非常好，非常适合入门) https://www.bilibili.com/video/BV1nR4y1E7Yj
6. GPS IMU 松组合 https://kth.instructure.com/files/677996/download?download_frd=1
7. Coursera课程 https://www.coursera.org/learn/state-estimation-localization-self-driving-cars 

推荐的学习路线：

1. 先看武汉大学惯性导航课程(牛小骥老师)，入门非常推荐，也不需要什么教材，做笔记
2. 看严恭敏老师的书籍，视频及code
3. 然后再入组合导航知识
