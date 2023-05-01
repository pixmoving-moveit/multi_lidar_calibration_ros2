# multi lidar calibration icp

## overview
- This ROS2 node of LiDAR 2 LiDAR extrinsic calibration.
- This ROS2 node is an implementation of [PCL regstration](https://pcl.readthedocs.io/projects/tutorials/en/latest/registration_api.html#registration-api) that using [ICP](https://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=&cad=rja&uact=8&ved=2ahUKEwiP7u7YqNP-AhVMCd4KHc61CJYQFnoECA8QAQ&url=https%3A%2F%2Fen.wikipedia.org%2Fwiki%2FIterative_closest_point&usg=AOvVaw0QHlhbU9_wC-E-cE_v13nG).
- Thanks [https://github.com/wutaoo] for this inspiration [multi_lidar_calibration](https://github.com/wutaoo/multi_lidar_calibration).

## parameters

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



