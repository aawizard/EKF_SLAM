#include "catch_ros2/catch_ros2.hpp"
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/executor.hpp>

#include "nuturtle_control/srv/init_config.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <catch2/catch_test_macros.hpp>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

using namespace std::chrono_literals;

TEST_CASE("turtle_odom_test_node", "[initial_pose_srv]") {

  auto node = rclcpp::Node::make_shared("turtle_odom_test_node");


  // Create a client for the service we're looking for
  auto client = node->create_client<nuturtle_control::srv::InitConfig>("/initial_pose");
  rclcpp::Time start_time = rclcpp::Clock().now();
  bool service_result = false;
  while (
    rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(20))
  )
  {
    if (client->wait_for_service(0s)) {   
      service_result = true;
      break;
    }
    rclcpp::spin_some(node);
  }

  CHECK(service_result);
}

TEST_CASE("turtle_odom_test_node", "[tf_broadcaster]") {
  auto node = rclcpp::Node::make_shared("turtle_odom_test_node");
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, node, false);
  bool tf_result = false;
  rclcpp::Time start_time = rclcpp::Clock().now();
  while (
    rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(20))
  )
  {
    try {
      auto t = tf_buffer->lookupTransform("odom", "base_footprint", tf2::TimePointZero);
      std::cout << t.transform.translation.x << std::endl;
      std::cout << t.transform.translation.y << std::endl;
      std::cout << t.transform.translation.z << std::endl;
      if (t.transform.translation.x == 0 && t.transform.translation.y == 0 && t.transform.translation.z == 0 && t.transform.rotation.x == 0 && t.transform.rotation.y == 0 && t.transform.rotation.z == 0 && t.transform.rotation.w == 1) {
        tf_result = true;
        break;
      }
      break;
    } catch (tf2::TransformException &ex) {
      rclcpp::spin_some(node);
    }
  }

  CHECK(tf_result);

}