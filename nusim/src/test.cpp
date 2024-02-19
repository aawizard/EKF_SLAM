
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "turtlelib/diff_drive.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"

#include <rclcpp/qos.hpp>


using namespace std::chrono_literals;

class Scan : public rclcpp::Node
{
public:
  Scan()
  : Node("scan"), count_(0)
  {
    rclcpp::QoS scan_qos_policy = rclcpp::QoS(rclcpp::KeepLast(20)).durability_volatile().best_effort();

    pub_scan_ = create_publisher<sensor_msgs::msg::LaserScan>("/scan", scan_qos_policy);


    timer_sensor_ = create_wall_timer(
      200ms, std::bind(&Scan::timer2_callback, this));

  }

private:


  void timer2_callback()
  {

    

      scan();
      pub_scan_->publish(scan_data);
    
  }

  void scan(){
    scan_data.header.frame_id = "odom";
    scan_data.header.stamp = get_clock()->now();
    scan_data.angle_min = 0;
    scan_data.angle_max = 2 * turtlelib::PI;
    scan_data.angle_increment = 0.01745329238474369;
    scan_data.time_increment = 0.0005592;
    scan_data.range_min = 0.11999999731779099;
    scan_data.range_max = 3.5;
    scan_data.scan_time = 0.2;
        //   scan_data.ranges.push_back(1.0);

    for(double i = scan_data.angle_min; i < scan_data.angle_max; i += scan_data.angle_increment){
      scan_data.ranges.push_back(i);
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer_sensor_;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_scan_;


  sensor_msgs::msg::LaserScan scan_data;

  size_t count_;

};

/// @brief    main function for the Scan node
/// @param argc
/// @param argv
/// @return
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Scan>());
  rclcpp::shutdown();
  return 0;
}
