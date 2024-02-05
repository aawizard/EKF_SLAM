#include "catch_ros2/catch_ros2.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;
auto freq = 0;
void cmd_sub(const geometry_msgs::msg::Twist::SharedPtr)
{
  freq++;
}


TEST_CASE("test_circle_node", "[frequency test]") {
  
  auto node = rclcpp::Node::make_shared("test_circle_node");
  node->declare_parameter<double>("test_duration");
  node->declare_parameter("frequency", 100);
  const auto TEST_DURATION =
    node->get_parameter("test_duration").get_parameter_value().get<double>();
  const auto test_freq = 
    node->get_parameter("frequency").as_int();
  
  auto subscriber = node->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10,&cmd_sub);


  rclcpp::Time start_time = rclcpp::Clock().now();
  // Keep test running only for the length of the "test_duration" parameter
  // (in seconds)
  while (
    rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(TEST_DURATION))
  )
  {
    
    rclcpp::spin_some(node);
  }

  // Test assertions - check that the dummy node was found
  REQUIRE_THAT(int(freq/TEST_DURATION), Catch::Matchers::WithinAbs(test_freq, 10));
}