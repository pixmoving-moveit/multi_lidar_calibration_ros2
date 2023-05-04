#include <multi_lidar_calibration/multi_lidar_calibration_icp.hpp>

int main(int argc, char ** argv)
{ 
  rclcpp::init(argc, argv);
  auto node = std::make_shared<calibration::multi_lidar_calibration_icp::MultiLidarCalibrationIcp>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}