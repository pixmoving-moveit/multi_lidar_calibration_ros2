#include <multi_lidar_calibration/multi_lidar_calibration_ndt_map.hpp>

namespace calibration
{
namespace multi_lidar_calibration_ndt_map
{

MultiLidarCalibrationNdtMap::MultiLidarCalibrationNdtMap()
: Node("multi_lidar_calibration_ndt_map")
{
  // initialize parameters of node
  param_.pcd_path = declare_parameter<std::string>("pcd_path", " ");
  param_.initial_pose =
    declare_parameter<std::vector<double>>("initial_pose", {0.0, 0.0, 0.0, 0.0, 1.57, 0.0});
  param_.leaf_size = declare_parameter<double>("leaf_size", 0.1);
  param_.max_iteration = declare_parameter<int>("max_iteration", 100);
  param_.transform_epsilon = declare_parameter<double>("transform_epsilon", 0.01);
  param_.step_size = declare_parameter<double>("step_size", 0.1);
  param_.resolution = declare_parameter<double>("resolution", 0.5);
  param_.max_coorespondence_distance =
    declare_parameter<double>("max_coorespondence_distance", 0.05);
  param_.euclidean_fitness_epsilon = declare_parameter<double>("euclidean_fitness_epsilon", 1.0);
  param_.ransac_outlier_rejection_threshold =
    declare_parameter<double>("ransac_outlier_rejection_threshold", 1.5); // ransac outlier rejection threshold
  param_.icp_max_iteration = declare_parameter<int>("icp_max_iteration", 100);
  // crop box filter parameters
  param_.crop_box_min_x = declare_parameter<double>("crop_box_min_x", -1.0); // crop box min x
  param_.crop_box_min_y = declare_parameter<double>("crop_box_min_y", -1.0); // crop box min y
  param_.crop_box_min_z = declare_parameter<double>("crop_box_min_z", -1.0); // crop box min z
  param_.crop_box_max_x = declare_parameter<double>("crop_box_max_x", 1.0); // crop box max x
  param_.crop_box_max_y = declare_parameter<double>("crop_box_max_y", 1.0); // crop box max y
  param_.crop_box_max_z = declare_parameter<double>("crop_box_max_z", 2.0); // crop box max z
  param_.icp_transform_epsilon = declare_parameter<double>("icp_transform_epsilon", 1e-9); // icp transform epsilon
  param_.negativate_crop_box = declare_parameter<bool>("negativate_crop_box", false); // crop box negation

  // sign
  is_source_pt_set_ = false;

  // load pcd file
  // source_pointcloud_ = new pcl::PointCloud<pcl::PointXYZI>;
  pcl::io::loadPCDFile<pcl::PointXYZI>(param_.pcd_path, source_pointcloud_);

  pointcloud_target_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "~/input/target_pointcloud", rclcpp::SensorDataQoS(),
    std::bind(&MultiLidarCalibrationNdtMap::callbackLidar, this, std::placeholders::_1));
   pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "~/output/cropped_pointcloud", rclcpp::SensorDataQoS());

  approximate_voxel_filter_.setLeafSize(param_.leaf_size, param_.leaf_size, param_.leaf_size);

  ndt_.setMaximumIterations(param_.max_iteration);
  ndt_.setTransformationEpsilon(param_.transform_epsilon);
  ndt_.setStepSize(param_.step_size);
  ndt_.setResolution(param_.resolution);

  // set ipc parameters
  icp_.setMaximumIterations(param_.icp_max_iteration);
  icp_.setTransformationEpsilon(param_.icp_transform_epsilon);
  icp_.setMaxCorrespondenceDistance(param_.max_coorespondence_distance);
  icp_.setEuclideanFitnessEpsilon(param_.euclidean_fitness_epsilon);
  icp_.setRANSACOutlierRejectionThreshold(param_.ransac_outlier_rejection_threshold);

  // set crop box parameters
  crop_box_filter_.setMin(Eigen::Vector4f(
    param_.crop_box_min_x, param_.crop_box_min_y, param_.crop_box_min_z, 0.0));
  crop_box_filter_.setMax(Eigen::Vector4f(
    param_.crop_box_max_x, param_.crop_box_max_y, param_.crop_box_max_z, 0.0));
  crop_box_filter_.setNegative(param_.negativate_crop_box);

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
  tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

}

MultiLidarCalibrationNdtMap::~MultiLidarCalibrationNdtMap()
{
}

void MultiLidarCalibrationNdtMap::callbackLidar(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
{
  rclcpp::Time start_time = this->now();
  
  pcl::PointCloud<pcl::PointXYZI>::Ptr target_pointcloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cropped_target_pointcloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr final_pointcloud (new pcl::PointCloud<pcl::PointXYZI>);

  // pcl::io::loadPCDFile<pcl::PointXYZI>(param_.pcd_path, *source_pointcloud);
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_source_pointcloud(
    new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_target_pointcloud(
    new pcl::PointCloud<pcl::PointXYZI>);

  pcl::fromROSMsg(*msg, *target_pointcloud);
  crop_box_filter_.setInputCloud(target_pointcloud);
  crop_box_filter_.filter(*cropped_target_pointcloud);
  sensor_msgs::msg::PointCloud2 cropped_points_msg;
  pcl::toROSMsg(*cropped_target_pointcloud, cropped_points_msg);
  cropped_points_msg.header = msg->header;
  pointcloud_publisher_->publish(cropped_points_msg);

  approximate_voxel_filter_.setInputCloud(cropped_target_pointcloud);
  approximate_voxel_filter_.filter(*filtered_target_pointcloud);

  ndt_.setInputSource(filtered_target_pointcloud);
  if(!is_source_pt_set_)
  {
    ndt_.setInputTarget(std::make_shared<pcl::PointCloud<pcl::PointXYZI>>(source_pointcloud_));
    is_source_pt_set_ = true;
  }
  if (ndt_iteration_count_ < 100)
  {
    ndt_.align(*final_pointcloud, current_transform_mtraix_);
    current_transform_mtraix_ = ndt_.getFinalTransformation();

    std::cout << "NDT converged." << std::endl
              << "The score is " << ndt_.getFitnessScore() << std::endl;
    std::cout << "Transformation matrix:" << std::endl;
    std::cout << current_transform_mtraix_ << std::endl;
    Eigen::Matrix3f rotation_matrix = current_transform_mtraix_.block(0, 0, 3, 3);
    Eigen::Vector3f translation_vector = current_transform_mtraix_.block(0, 3, 3, 1);
    std::cout << "This transformation can be replicated using:" << std::endl;
    std::cout << "ros2 run tf2_ros static_transform_publisher " << translation_vector.transpose()
              << " " << rotation_matrix.eulerAngles(2,1,0).transpose() << " " << "map"
              << " " << msg->header.frame_id.c_str() << std::endl;

    Eigen::Quaternionf q(rotation_matrix);
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = msg->header.stamp;
    t.header.frame_id = "map";
    t.child_frame_id = msg->header.frame_id;
    t.transform.translation.x = translation_vector[0];
    t.transform.translation.y = translation_vector[1];
    t.transform.translation.z = translation_vector[2];
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(t);
    ndt_iteration_count_++;
  }

  if (ndt_.hasConverged()&&ndt_iteration_count_ >= 100)
  {
    
    std::cout << "--------------------------------------" << std::endl;
    std::cout << "using icp to refine the transformation." << std::endl;
    icp_.setInputSource(filtered_target_pointcloud);
    icp_.setInputTarget(std::make_shared<pcl::PointCloud<pcl::PointXYZI>>(source_pointcloud_));
    icp_.align(*final_pointcloud, current_transform_mtraix_);
    current_transform_mtraix_ = icp_.getFinalTransformation();

    std::cout << "ICP converged." << std::endl
              << "The score is " << icp_.getFitnessScore() << std::endl;
    std::cout << "Transformation matrix:" << std::endl;
    std::cout << current_transform_mtraix_ << std::endl;
    Eigen::Matrix3f rotation_matrix = current_transform_mtraix_.block(0, 0, 3, 3);
    Eigen::Vector3f translation_vector = current_transform_mtraix_.block(0, 3, 3, 1);
    std::cout << "This transformation can be replicated using:" << std::endl;
    std::cout << "ros2 run tf2_ros static_transform_publisher " << translation_vector.transpose()
              << " " << rotation_matrix.eulerAngles(2,1,0).transpose() << " " << "map"
              << " " << msg->header.frame_id.c_str() << std::endl;

    Eigen::Quaternionf q(rotation_matrix);
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = msg->header.stamp;
    t.header.frame_id = "map";
    t.child_frame_id = msg->header.frame_id;
    t.transform.translation.x = translation_vector[0];
    t.transform.translation.y = translation_vector[1];
    t.transform.translation.z = translation_vector[2];
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(t);
  }
  rclcpp::Time end_time = this->now();
  std::cout << "process time: " << (end_time - start_time).seconds() * 1000.0 << "ms.\n";
}

} // namespace multi_lidar_calibration_ndt_map
} // calibration 