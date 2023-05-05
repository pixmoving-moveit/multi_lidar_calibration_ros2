#include <multi_lidar_calibration/point_cloud_filter.hpp>
namespace calibration
{
namespace filters_pass_through
{

PointCloudFilter::PointCloudFilter() : Node("point_cloud_filter")
{
  // ros2 param
  param_.input_topic_name = declare_parameter<std::string>("input_topic_name", "/input/point_cloud");
  param_.output_topic_name = declare_parameter<std::string>("output_topic_name", "/output/point_cloud");

  param_.min_x = declare_parameter<double>("min_x", 0);
  param_.max_x = declare_parameter<double>("max_x", 10);

  param_.min_y = declare_parameter<double>("min_y", -10);
  param_.max_y = declare_parameter<double>("max_y", 10);

  param_.min_z = declare_parameter<double>("min_z", -2);
  param_.max_z = declare_parameter<double>("max_z", 10);


  // Create publisher and subscriber
  sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      param_.input_topic_name, 10, std::bind(&PointCloudFilter::cloud_callback, this, std::placeholders::_1));

  pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(param_.output_topic_name, 10);
}

PointCloudFilter::~PointCloudFilter()
{
}

void PointCloudFilter::cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr input_cloud_msg)
{
  // Convert from ROS message to PCL point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input_cloud_msg, *input_cloud);

  // Create a PassThrough filter object
  pcl::PassThrough<pcl::PointXYZ> pass;

  // Filter along the x axis
  pass.setInputCloud(input_cloud);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(param_.min_x, param_.max_x);
  pass.filter(*input_cloud);

  // Filter along the y axis
  pass.setInputCloud(input_cloud);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(param_.min_y, param_.max_y);
  pass.filter(*input_cloud);

  // Filter along the z axis
  pass.setInputCloud(input_cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(param_.min_z, param_.max_z);
  pass.filter(*input_cloud);

  // Convert from PCL point cloud to ROS message
  sensor_msgs::msg::PointCloud2::SharedPtr output_cloud_msg(new sensor_msgs::msg::PointCloud2);
  pcl::toROSMsg(*input_cloud, *output_cloud_msg);

  // Publish the output cloud
  pub_->publish(*output_cloud_msg);
}

}
}