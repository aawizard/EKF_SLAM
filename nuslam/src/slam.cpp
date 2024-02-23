/// \file slam.cpp
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
///    \param green/path (nav_msgs::msg::Path): path of the robot
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
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/transform_broadcaster.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nuturtle_control/srv/init_config.hpp"
#include "turtlelib/diff_drive.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <tf2/transform_datatypes.h>

using namespace std::chrono_literals;
// using namespace turtlelib;

/// \brief subscribes to joint states and publishes odometry of the robot
/// \param body_id - body id of the robot
/// \param odom_id - odometry id of the robot
/// \param wheel_left - left wheel joint name
/// \param wheel_right - right wheel joint name
/// \param wheel_radius - radius of the wheels[m]
/// \param track_width - distance between the wheels[m]
class Slam : public rclcpp::Node
{
public:
  Slam()
  : Node("slam")
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
      exit(-1);
    }
    if (wheel_left == "") {
      RCLCPP_ERROR_STREAM(get_logger(), "wheel_left not set");
      exit(-1);
    }
    if (wheel_right == "") {
      RCLCPP_ERROR_STREAM(get_logger(), "wheel_right not set");
      exit(-1);
    }

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&Slam::odom_callback, this, std::placeholders::_1));

    // pub_odom_ = create_publisher<nav_msgs::msg::Slam>("odom", 10);
    pub_path_ = create_publisher<nav_msgs::msg::Path>("green/path", 10);
    timer_ = create_wall_timer(
      200ms, std::bind(&Slam::timer_callback, this));

    robot.initialize(track_width, wheel_radius, turtlelib::Transform2D());
    odom_robot_tf_.header.frame_id = odom_id;
    odom_robot_tf_.child_frame_id = body_id;
    odom_robot_tf_.header.stamp = this->get_clock()->now();
    map_odom_.header.frame_id = "map";
    map_odom_.child_frame_id = odom_id;
    map_odom_.header.stamp = this->get_clock()->now();
    state_ = {robot.get_robot_pos().rotation(), robot.get_robot_pos().translation().x, robot.get_robot_pos().translation().y};
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {

    // Get the current robot pose and twist
    auto q = msg->pose.pose.orientation;
    double yaw = euler_from_quaternion_yaw(q);
    // m.getRPY(roll, pitch, yaw);
    turtlelib::Twist2D twist{msg->twist.twist.linear.x, 0, yaw};
    turtlelib::Twist2D A = robot.diff_state(twist);
    
    // Update the robot pose
    robot.set_pos(turtlelib::Transform2D(turtlelib::Vector2D{msg->pose.pose.position.x, msg->pose.pose.position.y}, yaw));
    turtlelib::Transform2D Tob_ = robot.get_robot_pos();

    // Publish Transform
    odom_robot_tf_.header.stamp = this->get_clock()->now();
    odom_robot_tf_.transform.translation.x = Tob_.translation().x;
    odom_robot_tf_.transform.translation.y = Tob_.translation().y;
    odom_robot_tf_.transform.rotation = q;
    tf_broadcaster_->sendTransform(odom_robot_tf_);

    // Publish path
    robot_path_.header.stamp = this->get_clock()->now();
    robot_path_.header.frame_id = odom_id;
    geometry_msgs::msg::PoseStamped robot_pose_;    
    robot_pose_.header.stamp = this->get_clock()->now();
    robot_pose_.header.frame_id = odom_id;
    robot_pose_.pose.position.x = Tob_.translation().x;
    robot_pose_.pose.position.y = Tob_.translation().y;
    robot_pose_.pose.orientation = q;
    robot_path_.poses.push_back(robot_pose_);
    pub_path_->publish(robot_path_);
  }

  double euler_from_quaternion_yaw(geometry_msgs::msg::Quaternion quaternion){
    auto x = quaternion.x;
    auto y = quaternion.y;
    auto z = quaternion.z;
    auto w = quaternion.w;
    auto siny_cosp = 2 * (w * z + x * y);
    auto cosy_cosp = 1 - 2 * (y * y + z * z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);
    return yaw;
  }

  void timer_callback()
  {
    // Publish map to odom transform
    Tmb = Tmo * robot.get_robot_pos();
    state_ = {Tmb.rotation(), Tmb.translation().x, Tmb.translation().y};
    map_odom_.header.stamp = this->get_clock()->now();
    map_odom_.transform.translation.x = Tmo.translation().x;
    map_odom_.transform.translation.y = Tmo.translation().y;
    tf2::Quaternion q;
    q.setRPY(0, 0, Tmo.rotation());
    map_odom_.transform.rotation.x = q.x();
    map_odom_.transform.rotation.y = q.y();
    map_odom_.transform.rotation.z = q.z();
    map_odom_.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(map_odom_);
  }
  
  std::string body_id = "";
  std::string odom_id = "";
  std::string wheel_left = "";
  std::string wheel_right = "";
  double wheel_radius = 0.0;
  double track_width = 0.0;
  double left_wheel_joint = 0.0;
  double right_wheel_joint = 0.0;
  nav_msgs::msg::Odometry odom;
  turtlelib::Diff_drive robot;
  geometry_msgs::msg::TransformStamped odom_robot_tf_;
  geometry_msgs::msg::TransformStamped map_odom_;
  nav_msgs::msg::Path robot_path_;
  turtlelib::Transform2D Tmo;
  turtlelib::Transform2D Tmb;
  std::vector<double> state_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::TimerBase::SharedPtr timer_;

};

/// @brief    main function for the Slam node
/// @param argc
/// @param argv
/// @return
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Slam>());
  rclcpp::shutdown();
  return 0;
}
