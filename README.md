# multi lidar calibration icp

## overview
- This ROS2 node of LiDAR 2 LiDAR extrinsic calibration.
- This ROS2 node is an implementation of [PCL regstration](https://pcl.readthedocs.io/projects/tutorials/en/latest/registration_api.html#registration-api) that using [ICP](https://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=&cad=rja&uact=8&ved=2ahUKEwiP7u7YqNP-AhVMCd4KHc61CJYQFnoECA8QAQ&url=https%3A%2F%2Fen.wikipedia.org%2Fwiki%2FIterative_closest_point&usg=AOvVaw0QHlhbU9_wC-E-cE_v13nG) and [NDT]https://en.wikipedia.org/wiki/Normal_distributions_transform#:~:text=The%20normal%20distributions%20transform%20(NDT,working%20at%20University%20of%20Tübingen.).
- Thanks [https://github.com/wutaoo] for this inspiration [multi_lidar_calibration](https://github.com/wutaoo/multi_lidar_calibration).

## parameters

### multi_llidar_calibration_ndt

|*parameter*|*description*| *default value*|
|--|--|--|
|initial_pose| initial pose guess `[x, y, z, roll, pitch, yaw]` in radians | [0.0, 0.0, 0.0, 0.0, 1.57, 0.0] |
|max_iteration| max iteration for optimization | 100 |
|transform_epsilon| transform epsilon | 1e-9 |
|leaf_size| voxel leaf size of approximate voxel grip filter | 0.2 |
|step_size| step size of ndt registration | 0.05 |
|resolution| resolution of ndt registration | 0.5 |
|input/source_pointcloud| source pointcloud topic | /rs16/points |
|input/target_pointcloud| target pointcloud topic | /rsbp/points |

### multi_lidar_calibration_icp

|*parameter*|*description*| *default value*|
|--|--|--|
|initial_pose| initial pose guess `[x, y, z, roll, pitch, yaw]` in radians | [0.0, 0.0, 0.0, 0.0, 1.57, 0.0] |
|max_iteration| max iteration for optimization | 100 |
|transform_epsilon| transform epsilon | 1e-9 |
|max_coorespondence_distance| max coorespondence distance | 0.05 |
|euclidean_fitness_epsilon| euclidean fitness epsilon | 0.5 |
|ransac_outlier_rejection_threshold| RANSAC outlier rejection threshold | 1.1 |
|input/source_pointcloud| source pointcloud topic | /rs16/points |
|input/target_pointcloud| target pointcloud topic | /rsbp/points |

## How To Use

> There are 2 options for the multi-LiDAR extrinsics calibration, you can choose either ICP or NDT method. We are highly recommend you to use NDT method.

### Prerequisites

> if you have already installed Autoware.core/universe in your machine, you do not have to install these libraries below.

- Ubuntu 22.04
- [ROS2 Humble](https://docs.ros.org/en/humble/index.html)
- [PCL library](https://pointclouds.org)
- [pcl_ros](http://wiki.ros.org/pcl_ros)
- [tf2](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Tf2/Tf2-Main.html)
- [tf2_ros](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Tf2/Tf2-Main.html)

### Clone The Repository
```shell
mkdir calibration_ws
mkdir -p calibration_ws/src
cd calibration_ws/src
git clone https://github.com/pixmoving-moveit/multi_lidar_calibration_ros2.git
```

### Compile The Package

```shell
cd calibration_ws
colcon build
```

### Calibrate Multiple LiDARs

1. launch the ROS2 driver of multiple LiDARs
2. replace the `source_pointcloud` and `target_pointcloud` with you pointcloud topics parameters in the launch file.
3. launch calibrating program by executing the command below
```shell
ros2 launch multi_lidar_calibration multi_lidar_calbiration_ndt.launch.xml
```
4. you can see the calibration result in rviz2 as well as in the terminal.

![lidar2lidar](./images/lidar2lidar.gif)

## Reference

- [lidar2lidar calibration](https://pixmoving-moveit.github.io/pixkit-documentation-en/sensor-calibration/LiDAR-LiDAR-calibration/)