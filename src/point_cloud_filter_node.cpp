#include <multi_lidar_calibration/point_cloud_filter.hpp>

int main(int argc, char ** argv)
{ 
  rclcpp::init(argc, argv);
  auto node = std::make_shared<calibration::filters_pass_through::PointCloudFilter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}