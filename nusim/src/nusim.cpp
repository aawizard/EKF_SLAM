#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2_ros/transform_broadcaster.h>
#include "nusim/srv/teleport.hpp"

using namespace std::chrono_literals;
class Nusim : public rclcpp::Node
{
public:
  Nusim()
  : Node("nusim"), count_(0)
  {
    this->declare_parameter("rate", 200);
    this->declare_parameter("x0", 0.0);
    this->declare_parameter("y0", 0.0);
    this->declare_parameter("theta0", 0.0);

    auto rate = this->get_parameter("rate").as_int();
    publisher_ = this->create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);

    //service
    reset_service = this->create_service<std_srvs::srv::Empty>(
      "~/reset", std::bind(
        &Nusim::reset_callback, this,
        std::placeholders::_1, std::placeholders::_2));

    teleport_service = this->create_service<nusim::srv::Teleport>(
      "~/teleport", std::bind(
        &Nusim::teleport_callback, this,
        std::placeholders::_1, std::placeholders::_2));

    //timer
    timer_ = this->create_wall_timer(
      1000ms / rate, std::bind(&Nusim::timer_callback, this));
    //tf
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    //setting values of x,y,z

    x_ = this->get_parameter("x0").as_double();
    y_ = this->get_parameter("y0").as_double();
    theta_ = this->get_parameter("theta0").as_double();

    change_position(x_, y_, theta_);
  }

private:
  void reset_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response)
  {
    count_ = 0;
    change_position(0.0, 0.0, 0.0);
    RCLCPP_INFO(this->get_logger(), "Resetting count");
  }

  void change_position(double x, double y, double theta)
  {
    t.header.frame_id = "nusim/world";
    t.child_frame_id = "red/base_footprint";
    x_ = x;
    y_ = y;
    theta_ = theta;
    t.header.stamp = this->get_clock()->now();
    t.transform.translation.x = x;
    t.transform.translation.y = y;
    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(t);
  }

  void teleport_callback(
    const std::shared_ptr<nusim::srv::Teleport::Request> request,
    std::shared_ptr<nusim::srv::Teleport::Response> response)
  {
    x_ = request->x;
    y_ = request->y;
    theta_ = request->theta;
    RCLCPP_INFO(this->get_logger(), "Teleporting robot");
    change_position(x_, y_, theta_);
  }

  void timer_callback()
  {
    auto message = std_msgs::msg::UInt64();
    message.data = count_++;
    // RCLCPP_INFO(this->get_logger(), "Publishing: '%ld'", message.data);
    publisher_->publish(message);
  }
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service;
  rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport_service;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher_;
  geometry_msgs::msg::TransformStamped t;
  size_t count_;
  double x_ = 0.0;
  double y_ = 0.0;
  double theta_ = 0.0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nusim>());
  rclcpp::shutdown();
  return 0;
}