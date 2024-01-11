#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"

using namespace std::chrono_literals;
class Nusim : public rclcpp::Node
{
public:
  Nusim()
  : Node("nusim"), count_(0)
  {
    this->declare_parameter("rate", 200);
    auto rate = this->get_parameter("rate").as_int();
    publisher_ = this->create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
    reset_service = this->create_service<std_srvs::srv::Empty>(
      "~/reset", std::bind(
        &Nusim::reset_callback, this,
        std::placeholders::_1, std::placeholders::_2));

    timer_ = this->create_wall_timer(
      1000ms / rate, std::bind(&Nusim::timer_callback, this));
  }

private:
  void reset_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response)
  {
    count_ = 0;
    RCLCPP_INFO(this->get_logger(), "Resetting count");
  }

  void timer_callback()
  {
    auto message = std_msgs::msg::UInt64();
    message.data = count_++;
    RCLCPP_INFO(this->get_logger(), "Publishing: '%ld'", message.data);
    publisher_->publish(message);
  }
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nusim>());
  rclcpp::shutdown();
  return 0;
}