/// \file turtle_control.cpp
/// \brief Publishes wheel commands and joint states of the robot
/// \brief Subscribes to twist and calculates wheel commands.
/// \brief Subscribes to sensor data and calculates joint states.
///
///PARAMETERS:
///    \param wheel_radius (double): radius of the wheel [m]
///    \param track_width (double): distance between the wheels [m]
///    \param motor_cmd_max (double): maximum motor command [rad/s]
///    \param motor_cmd_per_rad_sec (double): motor command per rad/s
///    \param encoder_ticks_per_rev (double): encoder ticks per revolution
///    \param collision_radius (double): collision radius of the robot [m]
///SUBSCRIBES:
///    \param cmd_vel (geometry_msgs::msg::Twist): Twist command to execute
///    \param sensor_data (nuturtlebot_msgs::msg::SensorData): Sensor data from the robot
///PUBLISHES:
///    \param wheel_cmd (nuturtlebot_msgs::msg::WheelCommands): Wheel commands to execute
///    \param joint_states (sensor_msgs::msg::JointState): Joint states of the robot


#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "turtlelib/diff_drive.hpp"

using namespace std::chrono_literals;
// using namespace turtlelib;

/// \brief Publishes wheel commands and joint states of the robot
/// \param wheel_radius - radius of the wheel [m]
/// \param track_width - distance between the wheels [m]
/// \param motor_cmd_max - maximum motor command [rad/s]
/// \param motor_cmd_per_rad_sec - motor command per rad/s
/// \param encoder_ticks_per_rev - encoder ticks per revolution
/// \param collision_radius - collision radius of the robot [m]
class Turtle_control : public rclcpp::Node
{
public:
  Turtle_control()
  : Node("turtle_control")
  {
    declare_parameter("wheel_radius", -1.0);
    declare_parameter("track_width", -1.0);
    declare_parameter("motor_cmd_max", -1.0);
    declare_parameter("motor_cmd_per_rad_sec", -1.0);
    declare_parameter("encoder_ticks_per_rev", -1.0);
    declare_parameter("collision_radius", -1.0);

    wheel_radius = get_parameter("wheel_radius").as_double();
    track_width = get_parameter("track_width").as_double();
    motor_cmd_max = get_parameter("motor_cmd_max").as_double();
    motor_cmd_per_rad_sec = get_parameter("motor_cmd_per_rad_sec").as_double();
    encoder_ticks_per_rev = get_parameter("encoder_ticks_per_rev").as_double();
    collision_radius = get_parameter("collision_radius").as_double();

    if (wheel_radius < 0.0 || track_width < 0.0 || motor_cmd_max < 0.0 ||
      motor_cmd_per_rad_sec < 0.0 || encoder_ticks_per_rev < 0.0 || collision_radius < 0.0)
    {
      RCLCPP_ERROR_STREAM(get_logger(), "Invalid parameters");
      exit(-1);
    }

    sub_twist_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&Turtle_control::twist_callback, this, std::placeholders::_1));
    sub_sensor_ = create_subscription<nuturtlebot_msgs::msg::SensorData>(
      "sensor_data", 10, std::bind(&Turtle_control::sensor_callback, this, std::placeholders::_1));

    pub_wheel_ = create_publisher<nuturtlebot_msgs::msg::WheelCommands>("wheel_cmd", 10);
    pub_joint_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    robot.initilize(track_width, wheel_radius, turtlelib::Transform2D());
  }

private:
  void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    turtlelib::Twist2D twist{msg->angular.z, msg->linear.x, msg->linear.y};
    turtlelib::Wheel_state wheel_vels = robot.inverse_kinematics(twist);
    nuturtlebot_msgs::msg::WheelCommands wheel_cmd;
    wheel_cmd.left_velocity =int( wheel_vels.phi_l / motor_cmd_per_rad_sec);
    wheel_cmd.right_velocity = int(wheel_vels.phi_r / motor_cmd_per_rad_sec);

    if (wheel_cmd.left_velocity > motor_cmd_max) {
      wheel_cmd.left_velocity = motor_cmd_max;
    } else if (wheel_cmd.left_velocity < -motor_cmd_max) {
      wheel_cmd.left_velocity = -motor_cmd_max;
    }
    if (wheel_cmd.right_velocity > motor_cmd_max) {
      wheel_cmd.right_velocity = motor_cmd_max;
    } else if (wheel_cmd.right_velocity < -motor_cmd_max) {
      wheel_cmd.right_velocity = -motor_cmd_max;
    }
    pub_wheel_->publish(wheel_cmd);
  }
  void sensor_callback(const nuturtlebot_msgs::msg::SensorData::SharedPtr msg)
  {

    double left_wheel_joint = msg->left_encoder / encoder_ticks_per_rev;
    double right_wheel_joint = msg->right_encoder / encoder_ticks_per_rev;
    if (first_time) {
      joint_state.header.stamp = msg->stamp;
      joint_state.position = {0.0, 0.0};
      joint_state.velocity = {0.0, 0.0};
      first_time = false;
    }
    else{
      auto del_t = msg->stamp.sec + msg->stamp.nanosec * 1e-9 - joint_state.header.stamp.sec -
      joint_state.header.stamp.nanosec * 1e-9;
      del_t = del_t*1e5;
      RCLCPP_ERROR_STREAM(get_logger(), "js_left: " << joint_state.position[0]);
      RCLCPP_ERROR_STREAM(get_logger(), "js_right: " << joint_state.position[1]);
      double left_wheel_velocity = (left_wheel_joint - joint_state.position[0]) / del_t;
      double right_wheel_velocity = (right_wheel_joint - joint_state.position[1]) / del_t;
      RCLCPP_ERROR_STREAM(get_logger(), "Left wheel velocity: " << left_wheel_velocity);
      RCLCPP_ERROR_STREAM(get_logger(), "Right wheel velocity: " << right_wheel_velocity);
      //JointState calculation
      joint_state.header.stamp = msg->stamp;
      joint_state.position = {left_wheel_joint, right_wheel_joint};
      joint_state.velocity = {left_wheel_velocity, right_wheel_velocity};
      pub_joint_->publish(joint_state);
    }
  }

  double wheel_radius = 0.0;
  double track_width = 0.0;
  double motor_cmd_max = 0.0;
  double motor_cmd_per_rad_sec = 0.0;
  double encoder_ticks_per_rev = 0.0;
  double collision_radius = 0.0;
  bool first_time = true;
  turtlelib::Diff_drive robot;
  turtlelib::Wheel_state wheel_state;
  sensor_msgs::msg::JointState joint_state;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr sub_sensor_;
  rclcpp::Publisher<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr pub_wheel_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_;

};

/// @brief    main function for the turtle_control node
/// @param argc
/// @param argv
/// @return
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Turtle_control>());
  rclcpp::shutdown();
  return 0;
}
