#include <multi_lidar_calibration/multi_lidar_calibration_icp.hpp>

namespace calibration
{
namespace multi_lidar_calibration_icp
{

MultiLidarCalibrationIcp::MultiLidarCalibrationIcp()
: Node("multi_lidar_calibration_icp"),
  pointcloud_source_sub_(this, "~/input/source_pointcloud", rmw_qos_profile_sensor_data),
  pointcloud_target_sub_(this, "~/input/target_pointcloud", rmw_qos_profile_sensor_data),
  sync_(SyncPolicy(10), pointcloud_source_sub_, pointcloud_target_sub_)
{
  sync_.setMaxIntervalDuration(rclcpp::Duration(1, 0));
  // initialize parameters of node
  param_.initial_pose =
    declare_parameter<std::vector<double>>("initial_pose", {0.0, 0.0, 0.0, 0.0, 1.57, 0.0});
  param_.max_iteration = declare_parameter<int>("max_iteration", 100);
  param_.transform_epsilon = declare_parameter<double>("transform_epsilon", 1e-9);
  param_.max_coorespondence_distance =
    declare_parameter<double>("max_coorespondence_distance", 0.05);
  param_.euclidean_fitness_epsilon = declare_parameter<double>("euclidean_fitness_epsilon", 1.0);
  param_.ransac_outlier_rejection_threshold =
    declare_parameter<double>("ransac_outlier_rejection_threshold", 1.5);

  approximate_voxel_filter_.setLeafSize(0.2, 0.2, 0.2);
  
  icp_.setMaximumIterations(param_.max_iteration);
  icp_.setTransformationEpsilon(param_.transform_epsilon);
  icp_.setMaxCorrespondenceDistance(param_.max_coorespondence_distance);
  icp_.setEuclideanFitnessEpsilon(param_.euclidean_fitness_epsilon);
  icp_.setRANSACOutlierRejectionThreshold(param_.ransac_outlier_rejection_threshold);

  Eigen::Translation3f initial_translation(
    param_.initial_pose.at(0), param_.initial_pose.at(1), param_.initial_pose.at(2));

  Eigen::AngleAxisf initial_rotation_x(param_.initial_pose.at(3), Eigen::Vector3f::UnitX());
	Eigen::AngleAxisf initial_rotation_y(param_.initial_pose.at(4), Eigen::Vector3f::UnitY());
	Eigen::AngleAxisf initial_rotation_z(param_.initial_pose.at(5), Eigen::Vector3f::UnitZ());
  current_transform_mtraix_ =
    (initial_translation * initial_rotation_x * initial_rotation_y * initial_rotation_z)
      .matrix();
  std::cout << "initial guess: " << std::endl << current_transform_mtraix_ << std::endl;

  // tf2 broadcaster
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  sync_.registerCallback(std::bind(
    &MultiLidarCalibrationIcp::callbackLidars, this, std::placeholders::_1,
    std::placeholders::_2));
}

MultiLidarCalibrationIcp::~MultiLidarCalibrationIcp()
{
}

void MultiLidarCalibrationIcp::callbackLidars(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & point_1,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & point_2)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr source_pointcloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr target_pointcloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr final_pointcloud (new pcl::PointCloud<pcl::PointXYZI>);

  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_source_pointcloud(
    new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_target_pointcloud(
    new pcl::PointCloud<pcl::PointXYZI>);

  pcl::fromROSMsg(*point_1, *source_pointcloud);
  pcl::fromROSMsg(*point_2, *target_pointcloud);

  approximate_voxel_filter_.setInputCloud(target_pointcloud);
  approximate_voxel_filter_.filter(*filtered_target_pointcloud);

  icp_.setInputSource(filtered_target_pointcloud);
  icp_.setInputTarget(source_pointcloud);

  icp_.align(*final_pointcloud, current_transform_mtraix_);

  if (icp_.hasConverged())
  {
    current_transform_mtraix_ = icp_.getFinalTransformation();
    std::cout << "ICP converged." << std::endl
              << "The score is " << icp_.getFitnessScore() << std::endl;
    std::cout << "Transformation matrix:" << std::endl;
    std::cout << current_transform_mtraix_ << std::endl;
    Eigen::Matrix3f rotation_matrix = current_transform_mtraix_.block(0, 0, 3, 3);
    Eigen::Vector3f translation_vector = current_transform_mtraix_.block(0, 3, 3, 1);
    std::cout << "This transformation can be replicated using:" << std::endl;
    std::cout << "ros2 run tf2_ros static_transform_publisher " << translation_vector.transpose()
              << " " << rotation_matrix.eulerAngles(2,1,0).transpose() << " " << point_1->header.frame_id.c_str() 
              << " " << point_2->header.frame_id.c_str() << std::endl;

    Eigen::Quaternionf q(rotation_matrix);
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = point_1->header.stamp;
    t.header.frame_id = point_1->header.frame_id;
    t.child_frame_id = point_2->header.frame_id;
    t.transform.translation.x = translation_vector[0];
    t.transform.translation.y = translation_vector[1];
    t.transform.translation.z = translation_vector[2];
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(t);
  }
}

} // namespace multi_lidar_calibration_icp
} // calibration 