# lidar_undistortion_2d

> #### a ros package for 2d lidar motion compensation

## Introduction

读取odom数据对2D激光雷达数据进行运动畸变校正。

This ros package uses odom transform data to correct motion distortion of a 2D LIDAR in real time。

## Result

![](media/2022-10-1816-57-53屏幕截图.png)

在图片中，红色点云代表原始的激光雷达数据，白色点云代表经过运动补偿后的激光雷达数据。

in this picture, the yellow rectangle represents the pose of robot, the red poindcloud represents the origin lidar data, and the white pointcloud represents the lidar data after compensation.

## Parameters in launch file

名称 | 类型 |  注释
-------- | ----- | -----
scan_sub_topic | string | 订阅的2d激光数据话题名 
scan_pub_topic  | string | 经过运动畸变矫正后发布的2d激光数据话题名 
enable_pub_pointcloud  | bool | 是否将校正后的数据重新封装为2d LaserScan消息发布 
pointcloud_pub_topic | bool | 经过运动畸变矫正重新封装3d pointcloud消息话题名 
lidar_frame| string | 激光雷达数据的坐标系
odom_frame | string | Odometry数据的坐标系
lidar_scan_time_gain | double | 激光雷达单次扫描时间系数（正常情况下是1.0，但是有些激光雷达的驱动包在计算scan_time时有问题，所以这里乘一个系数）

## Test with rosbag
1. compile the project and `source devel/setup.sh`
2. execute the following command
```
roslaunch lidar_undistortion_2d lidar_undistortion_2d.launch
```
3. find `/bag/*8*.bag`
```
rosbag play --clock --pause ***.bag
```
remind: '--pause' is essential. otherwise it may lead to error. 

​	4.你可以选择打开pcl可视化或者自行进行rviz订阅节点消息

![](media/2022-10-1817-00-31屏幕截图.png)

## Reference

https://github.com/LiuLimingCode/lidar_undistortion_2d

https://github.com/elewu/2d_lidar_undistortion

深蓝学院SLAM教程
