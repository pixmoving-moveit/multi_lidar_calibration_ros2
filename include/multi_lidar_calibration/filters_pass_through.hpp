#ifndef __FLITER_PASS_THROUGH__HPP__
#define __FLITER_PASS_THROUGH__HPP__

// system library
#include <string>


// ros2 library
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

// pcl
#include <pcl/filters/passthrough.h>

namespace calibration
{
namespace filters_pass_through
{
  
  struct Param
  {
    std::string input_topic_name;
    std::string output_topic_name;
    double min_x;
    double max_x;
    double min_y;
    double max_y;
    double min_z;
    double max_z;
  };
  class PointCloudFilter: public rclcpp::Node
  {
  public:
    PointCloudFilter();
    ~PointCloudFilter();
  
  private:
    Param param_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;


    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr input_cloud_msg);
  };
  
}
}


#endif // __FLITER_PASS_THROUGH__HPP__ 
