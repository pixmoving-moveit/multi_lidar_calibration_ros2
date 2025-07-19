#ifndef __MULTI_LIDAR_CALIBRATION_NDT_MAP__HPP__
#define __MULTI_LIDAR_CALIBRATION_NDT_MAP__HPP__

#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/static_transform_broadcaster.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/crop_box.h>


namespace calibration
{
namespace multi_lidar_calibration_ndt_map
{

struct Param
{
  // std::string source_pointcloud_topic; // pointlcoud source topic
  // std::string target_pointcloud_topic; // pointcloud target topic
  std::string pcd_path;
  std::vector<double> initial_pose;  // x y z r p y
  // downsampling parameters
  double leaf_size;
  // ndt parameters
  int max_iteration;
  double transform_epsilon;
  double step_size;
  double resolution;
  // icp parameters
  double max_coorespondence_distance;
  double euclidean_fitness_epsilon;
  double ransac_outlier_rejection_threshold;
  int icp_max_iteration;
  double icp_transform_epsilon;

  // crop box parameters
  double crop_box_min_x;
  double crop_box_min_y;
  double crop_box_min_z;
  double crop_box_max_x;
  double crop_box_max_y;
  double crop_box_max_z;
  bool negativate_crop_box = false; // if true, crop box will be negated
};

class MultiLidarCalibrationNdtMap: public rclcpp::Node
{
private:
  Param param_;

  // subscriber
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_target_sub_;

  // downsampling filter
  pcl::ApproximateVoxelGrid<pcl::PointXYZI> approximate_voxel_filter_;

  // ndt registration
  pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt_;

  // icp registration
  pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp_;

  // crop box filter
  pcl::CropBox<pcl::PointXYZI> crop_box_filter_;

  // transform matrix
  Eigen::Matrix4f current_transform_mtraix_;

  // tf2
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
  pcl::PointCloud<pcl::PointXYZI> source_pointcloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
  bool is_source_pt_set_;
  int ndt_iteration_count_;

public:
  MultiLidarCalibrationNdtMap();
  ~MultiLidarCalibrationNdtMap();
  void callbackLidar(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);
};

}
}
#endif // __MULTI_LIDAR_CALIBRATION_NDT_MAP__HPP__ 