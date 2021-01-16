# 路径规划思路梳理

本项目的路径规划通过 `ROS Navigation` 实现，需要考虑以下内容：

- Transform 程序，发布 Topic `/tf`. 负责各个坐标系之间的转换。
- Odometry 程序，发布 Topic `/odom`. 负责发布里程计数据。
- 调整 `ROS Navigation` 的配置文件。
- 鉴于 `cartographer` 自带定位功能，且性能优越，弃用官方推荐的 `AMCL`。

## 里程计

可用数据源：雷达、相机、轮子

为方便调试，现阶段使用雷达和相机来生成里程计数据。

轮式里程计：姿态数据由 Cartographer 提供，速度由电机驱动提供。

