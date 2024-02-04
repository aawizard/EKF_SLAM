/// \file circle.cpp
/// \brief Publishes wheel commands and joint states of the robot
/// \brief Subscribes to twist and calculates wheel commands.
/// \brief Subscribes to sensor data and calculates joint states.
///
///PARAMETERS:
///    \param frequency (int): frequency of the node
///PUBLISHES:
///    \param cmd_vel (geometry_msgs::msg::Twist): Twist command to execute
///SERVICES:
///    \param control (nuturtle_control::srv::Control): set control parameters to make robot go in circle
///    \param reverse (std_srvs::srv::Empty): reverse the direction of the robot
///    \param stop (std_srvs::srv::Empty): stop the robot


#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "turtlelib/diff_drive.hpp"
#include "nuturtle_control/srv/control.hpp"
#include "std_srvs/srv/empty.hpp"

using namespace std::chrono_literals;

/// \brief State of the robot
/// \param MOVING - robot is moving
/// \param STOPPED - robot is stopped
enum class State
{
  MOVING,
  STOPPED
};

/// \brief Publishes wheel commands and joint states of the robot
/// \param frequency - frequency of the node
class Circle : public rclcpp::Node
{
public:
  Circle()
  : Node("circle")
  {
    declare_parameter("frequency", 100);
    frequency = get_parameter("frequency").as_int();

    timer_ = this->create_wall_timer(
      1000ms / frequency, std::bind(&Circle::timer_callback, this));

    control_service = this->create_service<nuturtle_control::srv::Control>(
      "control",
      std::bind(&Circle::control_callback, this, std::placeholders::_1, std::placeholders::_2));
    reverse_service = this->create_service<std_srvs::srv::Empty>(
      "reverse",
      std::bind(&Circle::reverse_callback, this, std::placeholders::_1, std::placeholders::_2));
    stop_service = this->create_service<std_srvs::srv::Empty>(
      "stop",
      std::bind(&Circle::stop_callback, this, std::placeholders::_1, std::placeholders::_2));
    pub_twist_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

private:
  void control_callback(
    nuturtle_control::srv::Control::Request::SharedPtr request,
    nuturtle_control::srv::Control::Response::SharedPtr)
  {
    twist.linear.x = request->radius * request->velocity;
    twist.angular.z = request->velocity;
    state = State::MOVING;
  }
  void reverse_callback(
    std_srvs::srv::Empty::Request::SharedPtr,
    std_srvs::srv::Empty::Response::SharedPtr)
  {
    twist.linear.x = -twist.linear.x;
    twist.angular.z = -twist.angular.z;
  }
  void stop_callback(
    std_srvs::srv::Empty::Request::SharedPtr,
    std_srvs::srv::Empty::Response::SharedPtr)
  {
    state = State::STOPPED;
    twist.linear.x = 0.0;
    twist.angular.z = 0.0;
    pub_twist_->publish(twist);
  }

  void timer_callback()
  {
    if (state == State::MOVING) {
      pub_twist_->publish(twist);
    }
  }


  int frequency = 100;
  double radius = 0.0;
  double velocity = 0.0;
  State state = State::STOPPED;
  geometry_msgs::msg::Twist twist;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Service<nuturtle_control::srv::Control>::SharedPtr control_service;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reverse_service;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_service;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_twist_;
};

/// @brief    main function for the Circle node
/// @param argc
/// @param argv
/// @return
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Circle>());
  rclcpp::shutdown();
  return 0;
}
