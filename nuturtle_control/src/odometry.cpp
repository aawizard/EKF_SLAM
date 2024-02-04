/// \file odometry.cpp
/// \brief TODO: Fill in the documentation for this file
///
///PARAMETERS:
///    \param body_id (string): body id of the robot
///    \param odom_id (string): odometry id of the robot
///    \param wheel_left (string): left wheel joint name
///    \param wheel_right (string): right wheel joint name
///PUBLISHES:
///    \param wheel_cmd (nuturtlebot_msgs::msg::WheelCommands): Wheel commands to execute
///    \param joint_states (sensor_msgs::msg::JointState): Joint states of the robot


#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
// #include "turtlelib/se2d.hpp"
// #include "turtlelib/geometry.hpp"
#include "turtlelib/diff_drive.hpp"

using namespace std::chrono_literals;
// using namespace turtlelib;

/// \brief TODO: Fill in the documentation for this class
/// \param body_id - body id of the robot
/// \param odom_id - odometry id of the robot
/// \param wheel_left - left wheel joint name
/// \param wheel_right - right wheel joint name
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

    body_id = get_parameter("body_id").as_string();
    odom_id = get_parameter("odom_id").as_string();
    wheel_left = get_parameter("wheel_left").as_string();
    wheel_right = get_parameter("wheel_right").as_string();

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
    sub_joint_state_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(&Odometry::joint_state_callback, this, std::placeholders::_1));
    // sub_sensor_ = create_subscription<nuturtlebot_msgs::msg::SensorData>(
    //   "sensor_data", 10, std::bind(&Odometry::sensor_callback, this, std::placeholders::_1));

    // pub_wheel_ = create_publisher<nuturtlebot_msgs::msg::WheelCommands>("wheel_cmd", 10);
    // pub_joint_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    // robot.initilize(track_width, wheel_radius, turtlelib::Transform2D());
    // joint_state.header.stamp = this->get_clock()->now();
    // joint_state.name = {"left_wheel_joint", "right_wheel_joint"};
    odom.header.frame_id = odom_id;
    odom.child_frame_id = body_id;
    odom.header.stamp = this->get_clock()->now();
  }

private:
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    { 
        auto left_wheel_joint = msg->position[0];
        auto right_wheel_joint = msg->position[1];
    }

//   void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
//   {
//     turtlelib::Twist2D twist{msg->angular.z, msg->linear.x, msg->linear.y};
//     turtlelib::Wheel_state wheel_vels = robot.inverse_kinematics(twist);

//     //Diffdrive claculation
//     nuturtlebot_msgs::msg::WheelCommands wheel_cmd;
//     wheel_cmd.left_velocity = wheel_vels.phi_l / motor_cmd_per_rad_sec;
//     wheel_cmd.right_velocity = wheel_vels.phi_r / motor_cmd_per_rad_sec;

//     if (wheel_cmd.left_velocity > motor_cmd_max) {
//       wheel_cmd.left_velocity = motor_cmd_max;
//     } else if (wheel_cmd.left_velocity < -motor_cmd_max) {
//       wheel_cmd.left_velocity = -motor_cmd_max;
//     }
//     if (wheel_cmd.right_velocity > motor_cmd_max) {
//       wheel_cmd.right_velocity = motor_cmd_max;
//     } else if (wheel_cmd.right_velocity < -motor_cmd_max) {
//       wheel_cmd.right_velocity = -motor_cmd_max;
//     }
//     pub_wheel_->publish(wheel_cmd);
//   }
//   void sensor_callback(const nuturtlebot_msgs::msg::SensorData::SharedPtr msg)
//   {
//     double left_wheel_joint = msg->left_encoder / encoder_ticks_per_rev;
//     double right_wheel_joint = msg->right_encoder / encoder_ticks_per_rev;
//     auto del_t = msg->stamp.sec + msg->stamp.nanosec * 1e-9 - joint_state.header.stamp.sec -
//       joint_state.header.stamp.nanosec * 1e-9;
//     double left_wheel_velocity = (left_wheel_joint - joint_state.position[0]) / del_t;
//     double right_wheel_velocity = (right_wheel_joint - joint_state.position[1]) / del_t;
//     //JointState calculation
//     joint_state.header.stamp = this->get_clock()->now();
//     joint_state.position = {left_wheel_joint, right_wheel_joint};
//     joint_state.velocity = {left_wheel_velocity, right_wheel_velocity};
//     pub_joint_->publish(joint_state);
//   }

  std::string body_id = "";
  std::string odom_id = "";
  std::string wheel_left = "";
  std::string wheel_right = "";
  nav_msgs::msg::Odometry odom;
//   turtlelib::Diff_drive robot;
//   turtlelib::Wheel_state wheel_state;
//   sensor_msgs::msg::JointState joint_state;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_state_;
//   rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr sub_sensor_;
//   rclcpp::Publisher<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr pub_wheel_;
//   rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_;

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
