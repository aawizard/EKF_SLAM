/// \file
/// \brief This file contains the implementation of the nusim node
///         Nusim node act as a simulator for the turtlebot3.
///         It publishes the current time step and the current pose of the robot.
///         It also provides a service to reset the time step and teleport the robot to a new pose.
///         The arena for the robot is shown in rviz with walls and obstacles.
///PARAMETERS:
///     \param rate: the rate at which the time step is published [Hz]
///     \param x0: the initial x position of the robot [m]
///     \param y0: the initial y position of the robot [m]
///     \param theta0: the initial orientation of the robot [rad]
///     \param arena_x_length: the length of the arena in the x direction [m]
///     \param arena_y_length: the length of the arena in the y direction [m]
///     \param obstacles/x: the x positions of the obstacles [m]
///     \param obstacles/y: the y positions of the obstacles [m]
///     \param obstacles/radius: the radius of the obstacles [m]
///     \param wheel_radius: the radius of the wheels [m]
///     \param track_width: the distance between the wheels [m]
///     \param motor_cmd_per_rad_sec: the motor command per rad/s
///PUBLISHES:
///     pub topic: ~/timestep [std_msgs::msg::UInt64] the current time step
///     pub topic: ~/walls [visualization_msgs::msg::MarkerArray] the walls of the arena
///     pub topic: ~/obstacles [visualization_msgs::msg::MarkerArray] the obstacles in the arena
///     pub topic: red/sensor_data [nuturtlebot_msgs::msg::SensorData] the sensor data of the robot
///SERVICES:
///     srv service: ~/reset [std_srvs::srv::Empty] resets the time step and teleports the robot to the initial pose
///     srv service: ~/teleport [nusim::srv::Teleport] teleports the robot to a new pose
///SUBSCRIBES:
///     sub topic: red/wheel_cmd [nuturtlebot_msgs::msg::WheelCommands] the wheel commands of the robot, publishes the sensor data and update robot position
///CLIENTS:
///     None
///BROADCASTS:
///     broadcaster tf: red/base_footprint to nusim/world


#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "turtlelib/diff_drive.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2_ros/transform_broadcaster.h>
#include "nusim/srv/teleport.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <nuturtlebot_msgs/msg/wheel_commands.hpp>
#include <nuturtlebot_msgs/msg/sensor_data.hpp>
#include <rclcpp/qos.hpp>

using namespace std::chrono_literals;


/// \brief This class publishes the current timestep of the simulation, obstacles and walls that
///        appear in Rviz as markers. The class has a timer_callback to continually update the
///        simulation at each timestep. The reset service resets the simulation to the initial
///        state thus restarting the simulation. A teleport service is available to teleport a
///        turtlebot to any pose. A broadcaster broadcasts the robots TF frames to a topic for
///        visualization in Rviz.
/// \param rate: the rate at which the time step is published [Hz]
/// \param x0: the initial x position of the robot [m]
/// \param y0: the initial y position of the robot [m]
/// \param theta0: the initial orientation of the robot [rad]
/// \param arena_x_length: the length of the arena in the x direction [m]
/// \param arena_y_length: the length of the arena in the y direction [m]
/// \param obstacles/x: the x positions of the obstacles [m]
/// \param obstacles/y: the y positions of the obstacles [m]
/// \param obstacles/radius: the radius of the obstacles [m]
/// \param wheel_radius: the radius of the wheels [m]
/// \param track_width: the distance between the wheels [m]
/// \param motor_cmd_per_rad_sec: the motor command per rad/s
class Nusim : public rclcpp::Node
{
public:
  Nusim()
  : Node("nusim"), count_(0)
  {
    declare_parameter("rate", 200);
    declare_parameter("x0", 0.0);
    declare_parameter("y0", 0.0);
    //parameter for wall
    declare_parameter("theta0", 0.0);
    declare_parameter("arena_x_length", 2.0);
    declare_parameter("arena_y_length", 2.0);

    //parameters for obstacles
    declare_parameter("obstacles/x", obstacle_x_);
    declare_parameter("obstacles/y", obstacle_y_);
    declare_parameter("obstacles/radius", 0.05);
    declare_parameter("wheel_radius", 0.033);
    declare_parameter("track_width", 0.160);
    declare_parameter("motor_cmd_per_rad_sec", 0.024);
    rate = get_parameter("rate").as_int();
    rclcpp::QoS qos_policy = rclcpp::QoS(rclcpp::KeepLast(10)).transient_local();
    //publisher
    publisher_timestep_ = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
    publisher_walls_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/walls",
      qos_policy);
    publisher_obstacles_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/obstacles",
      qos_policy);
    pub_sensor_ = create_publisher<nuturtlebot_msgs::msg::SensorData>("red/sensor_data", 10);
    //subscriber
    sub_wheel_ = create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
      "red/wheel_cmd", 10, std::bind(&Nusim::wheel_cmd_callback, this, std::placeholders::_1)
    );
    //service
    reset_service = create_service<std_srvs::srv::Empty>(
      "~/reset", std::bind(
        &Nusim::reset_callback, this,
        std::placeholders::_1, std::placeholders::_2));

    teleport_service = create_service<nusim::srv::Teleport>(
      "~/teleport", std::bind(
        &Nusim::teleport_callback, this,
        std::placeholders::_1, std::placeholders::_2));

    //timer
    timer_ = create_wall_timer(
      1000ms / rate, std::bind(&Nusim::timer_callback, this));
    //tf
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    //setting values of x,y,z
    x0_ = get_parameter("x0").as_double();
    y0_ = get_parameter("y0").as_double();
    theta0_ = get_parameter("theta0").as_double();
    wheel_radius_ = get_parameter("wheel_radius").as_double();
    track_width_ = get_parameter("track_width").as_double();
    motor_cmd_per_rad_sec_ = get_parameter("motor_cmd_per_rad_sec").as_double();
    x_ = x0_;
    y_ = y0_;
    theta_ = theta0_;
    arena_x_length_ = get_parameter("arena_x_length").as_double();
    arena_y_length_ = get_parameter("arena_y_length").as_double();
    obstacle_x_ = get_parameter("obstacles/x").as_double_array();
    obstacle_y_ = get_parameter("obstacles/y").as_double_array();
    std::vector<double> k = get_parameter("obstacles/x").as_double_array();
    obstacle_radius_ = get_parameter("obstacles/radius").as_double();
    change_position(x_, y_, theta_);
    robot_.initialize(
      track_width_, wheel_radius_,
      turtlelib::Transform2D(turtlelib::Vector2D{x_, y_}, theta_));

    show_walls();
    make_obstacles();
  }

private:
  /// \brief callback for the reset service
  void reset_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    count_ = 0;
    change_position(x0_, y0_, theta0_);
    RCLCPP_INFO(get_logger(), "Resetting count");
  }
  /// \brief callback for the wheel command subscriber
  /// \param msg [nuturtlebot_msgs::msg::WheelCommands] the wheel commands of the robot
  void wheel_cmd_callback(const nuturtlebot_msgs::msg::WheelCommands::SharedPtr msg)
  {
    //Save the wheel commands
    left_wheel_joint += static_cast<double>(msg->left_velocity);
    right_wheel_joint += static_cast<double>(msg->right_velocity);
    //Change to sensor data
    nuturtlebot_msgs::msg::SensorData sensor_data;
    sensor_data.left_encoder = left_wheel_joint;
    sensor_data.right_encoder = right_wheel_joint;
    sensor_data.stamp = get_clock()->now();

    //Publish the sensor data
    pub_sensor_->publish(sensor_data);

    //reverse the wheel commands to get the robot to move

    wheel_vels.phi_l = (msg->left_velocity * motor_cmd_per_rad_sec_) / rate;
    wheel_vels.phi_r = (msg->right_velocity * motor_cmd_per_rad_sec_) / rate;
    //DO FK
    robot_.forward_kinematics(wheel_vels);
    //Change x_,y_,theta_
    x_ = robot_.get_robot_pos().translation().x;
    y_ = robot_.get_robot_pos().translation().y;
    theta_ = robot_.get_robot_pos().rotation();
    //Change the position of tf
    change_position(x_, y_, theta_);

  }

  /// \brief changes the position of the robot
  /// \param x new x position of the robot
  /// \param y new y position of the robot
  /// \param theta new orientation of the robot
  void change_position(double x, double y, double theta)
  {
    x_ = x;
    y_ = y;
    theta_ = theta;
    t.header.frame_id = "nusim/world";
    t.child_frame_id = "red/base_footprint";
    t.header.stamp = get_clock()->now();
    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    // tf_broadcaster_->sendTransform(t);
  }
  /// \brief callback for the teleport service to teleport the robot to a new pose
  /// \param request [nusim/srv/Teleport/Request] the request for the service
  /// \param response [nusim/srv/Teleport/Response] the response for the service
  void teleport_callback(
    const std::shared_ptr<nusim::srv::Teleport::Request> request,
    std::shared_ptr<nusim::srv::Teleport::Response>)
  {
    x_ = request->x;
    y_ = request->y;
    theta_ = request->theta;
    RCLCPP_INFO(get_logger(), "Teleporting robot");
    change_position(x_, y_, theta_);
  }
  /// \brief callback for the timer to publish the current time step
  void timer_callback()
  {
    auto message = std_msgs::msg::UInt64();
    message.data = count_++;
    // RCLCPP_INFO_STREAM(get_logger(), "Publishing: '" << message.data << "'");
    publisher_timestep_->publish(message);
    t.header.stamp = get_clock()->now();
    tf_broadcaster_->sendTransform(t);

  }
  /// \brief creates a wall marker
  visualization_msgs::msg::Marker make_wall(int id, double scale[], const double pose[])
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "nusim/world";
    marker.header.stamp = get_clock()->now();
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = scale[0];
    marker.scale.y = scale[1];
    marker.scale.z = scale[2];

    marker.pose.position.x = pose[0];
    marker.pose.position.y = pose[1];
    marker.pose.position.z = pose[2];
    marker.color.r = 1.0;
    marker.color.a = 1.0;
    marker.id = id;
    marker.frame_locked = true;
    return marker;

  }
  /// \brief creates an obstacle marker
  void make_obstacles()
  {
    auto obstacles = visualization_msgs::msg::MarkerArray();
    auto obstacle = visualization_msgs::msg::Marker();
    if (obstacle_x_.size() != obstacle_y_.size()) {
      RCLCPP_ERROR_STREAM(get_logger(), "Obstacle x and y vectors are not the same size");
      return;
    }
    for (int i = 0; i < int(obstacle_x_.size()); ++i) {
      obstacle.header.frame_id = "nusim/world";
      obstacle.header.stamp = get_clock()->now();
      obstacle.type = visualization_msgs::msg::Marker::CYLINDER;
      obstacle.action = visualization_msgs::msg::Marker::ADD;
      obstacle.color.r = 1.0;
      obstacle.color.a = 1.0;
      obstacle.id = i + 4;
      obstacle.frame_locked = true;
      obstacle.scale.x = obstacle_radius_ * 2;
      obstacle.scale.y = obstacle_radius_ * 2;
      obstacle.scale.z = wall_height;
      obstacle.pose.position.x = obstacle_x_[i];
      obstacle.pose.position.y = obstacle_y_[i];
      obstacle.pose.position.z = wall_height / 2;

      obstacles.markers.push_back(obstacle);
    }
    publisher_obstacles_->publish(obstacles);
  }
  /// \brief creates a wall marker
  void show_walls()
  {
    auto walls = visualization_msgs::msg::MarkerArray();
    auto wall = visualization_msgs::msg::Marker();
    double scale[3] = {0.0, 0.0, 0.0};
    double pose[3] = {0.0, 0.0, 0.0};
    scale[2] = wall_height;
    pose[2] = wall_height / 2;
    for (int i = 0; i < 4; ++i) {
      if (i == 0 || i == 2) {
        //left
        scale[0] = arena_x_length_;
        scale[1] = wall_thickness;
        pose[0] = 0;
        pose[1] = arena_y_length_ / 2;
      }
      if (i == 2) {
        pose[1] *= -1;
      }
      if (i == 1 || i == 3) {
        scale[0] = wall_thickness;
        scale[1] = arena_y_length_;
        pose[0] = arena_x_length_ / 2;
        pose[1] = 0;
      }
      if (i == 3) {
        pose[0] *= -1;
      }
      wall = make_wall(i, scale, pose);
      walls.markers.push_back(wall);
    }
    publisher_walls_->publish(walls);
  }

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service;
  rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport_service;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher_timestep_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_walls_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_obstacles_;
  rclcpp::Publisher<nuturtlebot_msgs::msg::SensorData>::SharedPtr pub_sensor_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr sub_wheel_;
  geometry_msgs::msg::TransformStamped t = geometry_msgs::msg::TransformStamped();

  int rate = 200;
  double x_ = 0.0;
  double y_ = 0.0;
  double theta_ = 0.0;
  double x0_ = 0.0;
  double y0_ = 0.0;
  double theta0_ = 0.0;
  double wall_height = 0.25;
  double wall_thickness = 0.10;
  double arena_x_length_ = 1.0;
  double arena_y_length_ = 1.0;
  double left_wheel_joint = 0.0;
  double right_wheel_joint = 0.0;
  double wheel_radius_ = 0.0;
  double track_width_ = 0.0;
  double motor_cmd_per_rad_sec_ = 0.0;
  double encoder_ticks_per_rev_sec_ = 0.0;
  turtlelib::Diff_drive robot_;
  turtlelib::Wheel_state wheel_vels;

  std::vector<double> obstacle_x_ = {0.25, 0.35};
  std::vector<double> obstacle_y_ = {0.25, -0.25};
  float obstacle_radius_ = 0.05;
  size_t count_;

};

/// @brief    main function for the nusim node
/// @param argc
/// @param argv
/// @return
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nusim>());
  rclcpp::shutdown();
  return 0;
}
