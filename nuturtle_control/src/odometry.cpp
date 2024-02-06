/// \file odometry.cpp
/// \brief Subscribes to joint states and publishes odometry of the robot
///
///PARAMETERS:
///    \param body_id (string): body id of the robot
///    \param odom_id (string): odometry id of the robot
///    \param wheel_left (string): left wheel joint name
///    \param wheel_right (string): right wheel joint name
///    \param wheel_radius (double): radius of the wheels[m]
///    \param track_width (double): distance between the wheels[m]
///PUBLISHES:
///    \param odom (nav_msgs::msg::Odometry): odometry of the robot
///SUBSCRIBES:
///    \param joint_states (sensor_msgs::msg::JointState): joint states of the robot
///SERVICES:
///    \param initial_pose (nuturtle_control::srv::InitConfig): set initial pose of the robot

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nuturtle_control/srv/init_config.hpp"
#include "turtlelib/diff_drive.hpp"

using namespace std::chrono_literals;
// using namespace turtlelib;

/// \brief subscribes to joint states and publishes odometry of the robot
/// \param body_id - body id of the robot
/// \param odom_id - odometry id of the robot
/// \param wheel_left - left wheel joint name
/// \param wheel_right - right wheel joint name
/// \param wheel_radius - radius of the wheels[m]
/// \param track_width - distance between the wheels[m]
class Odometry : public rclcpp::Node
{
public:
  Odometry()
  : Node("odometry")
  {
    declare_parameter("body_id", "");
    declare_parameter("odom_id", "odom");
    declare_parameter("wheel_left", "");
    declare_parameter("wheel_right", "");
    declare_parameter("wheel_radius", -1.0);
    declare_parameter("track_width", -1.0);

    body_id = get_parameter("body_id").as_string();
    odom_id = get_parameter("odom_id").as_string();
    wheel_left = get_parameter("wheel_left").as_string();
    wheel_right = get_parameter("wheel_right").as_string();
    wheel_radius = get_parameter("wheel_radius").as_double();
    track_width = get_parameter("track_width").as_double();

    if (body_id == "") {
      RCLCPP_ERROR_STREAM(get_logger(), "body_id not set");
      // exit(-1);
    }
    if (wheel_left == "") {
      RCLCPP_ERROR_STREAM(get_logger(), "wheel_left not set");
      // exit(-1);
    }
    if (wheel_right == "") {
      RCLCPP_ERROR_STREAM(get_logger(), "wheel_right not set");
      // exit(-1);
    }

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    sub_joint_state_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(&Odometry::joint_state_callback, this, std::placeholders::_1));

    pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    init_config_service = this->create_service<nuturtle_control::srv::InitConfig>(
      "/initial_pose", std::bind(
        &Odometry::init_pose_callback, this,
        std::placeholders::_1, std::placeholders::_2));

    odom.header.frame_id = odom_id;
    odom.child_frame_id = body_id;
    odom.header.stamp = this->get_clock()->now();
    robot.initilize(track_width, wheel_radius, turtlelib::Transform2D());
    odom_tf_.header.frame_id = odom_id;
    odom_tf_.child_frame_id = body_id;
    odom_tf_.header.stamp = this->get_clock()->now();
  }

private:
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    turtlelib::Wheel_state wheel_vels;
    wheel_vels.phi_l = msg->position[0] - robot.get_wheel_pos().phi_l;
    wheel_vels.phi_r = msg->position[1] - robot.get_wheel_pos().phi_r;
    turtlelib::Twist2D twist = robot.Twist(wheel_vels);
    robot.forward_kinematics(wheel_vels);
    turtlelib::Transform2D Tsb_ = robot.get_robot_pos();
    odom.pose.pose.position.x = Tsb_.translation().x;
    odom.pose.pose.position.y = Tsb_.translation().y;
    tf2::Quaternion q;
    q.setRPY(0, 0, Tsb_.rotation());
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom.twist.twist.linear.x = twist.x;
    odom.twist.twist.angular.z = twist.omega;
    pub_odom_->publish(odom);
    odom_tf_.transform.translation.x = Tsb_.translation().x;
    odom_tf_.transform.translation.y = Tsb_.translation().y;
    odom_tf_.transform.rotation.x = q.x();
    odom_tf_.transform.rotation.y = q.y();
    odom_tf_.transform.rotation.z = q.z();
    odom_tf_.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(odom_tf_);
  }
  void init_pose_callback(
    const std::shared_ptr<nuturtle_control::srv::InitConfig::Request> request,
    std::shared_ptr<nuturtle_control::srv::InitConfig::Response>)
  {
    turtlelib::Transform2D Tsb({request->x, request->y}, request->theta);
    robot.set_pos(Tsb);
  }
  std::string body_id = "";
  std::string odom_id = "";
  std::string wheel_left = "";
  std::string wheel_right = "";
  double wheel_radius = 0.0;
  double track_width = 0.0;
  nav_msgs::msg::Odometry odom;
  turtlelib::Diff_drive robot;
  geometry_msgs::msg::TransformStamped odom_tf_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_state_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Service<nuturtle_control::srv::InitConfig>::SharedPtr init_config_service;

};

/// @brief    main function for the Odometry node
/// @param argc
/// @param argv
/// @return
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odometry>());
  rclcpp::shutdown();
  return 0;
}
