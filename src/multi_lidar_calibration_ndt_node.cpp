#include <multi_lidar_calibration/multi_lidar_calibration_ndt.hpp>

int main(int argc, char ** argv)
{ 
  rclcpp::init(argc, argv);
  auto node = std::make_shared<calibration::multi_lidar_calibration_ndt::MultiLidarCalibrationNdt>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}