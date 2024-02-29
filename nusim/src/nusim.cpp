/// \file
/// \brief This file contains the implementation of the nusim node
///         Nusim node act as a simulator for the turtlebot3.
///         It publishes the current time step and the current pose of the robot.
///         It also provides a service to reset the time step and teleport the robot to a new pose.
///         The arena for the robot is shown in rviz with walls and obstacles.
///         The robot simulates the sensor data and publishes it to the /fake_sensor topic.
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
///     \param encoder_ticks_per_rev_sec: the encoder ticks per revolution per second[Hz]
///     \param draw_only: if true, only draw the arena and do not simulate the robot
///     \param input_noise: the noise in the input
///     \param slip_fraction: the fraction of slip in the robot
///     \param collision_radius: the radius of the robot for collision detection
///     \param max_range: the maximum range of the lidar sensor
///     \param basic_sensor_variance: the variance of the basic sensor
///     \param laser_min_range: the minimum range of the lidar sensor[m]
///     \param laser_max_range: the maximum range of the lidar sensor[m]
///PUBLISHES:
///     pub topic: ~/timestep [std_msgs::msg::UInt64] the current time step
///     pub topic: ~/walls [visualization_msgs::msg::MarkerArray] the walls of the arena
///     pub topic: ~/obstacles [visualization_msgs::msg::MarkerArray] the obstacles in the arena
///     pub topic: red/sensor_data [nuturtlebot_msgs::msg::SensorData] the sensor data of the robot
///     pub topic: red/path [nav_msgs::msg::Path] the path of the robot
///     pub topic: /fake_sensor [visualization_msgs::msg::MarkerArray] the fake sensor data of the robot
///     pub topic: /scan [sensor_msgs::msg::LaserScan] the scan data of the robot
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
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2_ros/transform_broadcaster.h>
#include "nusim/srv/teleport.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <nuturtlebot_msgs/msg/wheel_commands.hpp>
#include <nuturtlebot_msgs/msg/sensor_data.hpp>
#include <rclcpp/qos.hpp>
#include <nav_msgs/msg/path.hpp>
#include <random>


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
/// \param encoder_ticks_per_rev_sec: the encoder ticks per revolution per second[Hz]
/// \param draw_only: if true, only draw the arena and do not simulate the robot
/// \param input_noise: the noise in the input
/// \param slip_fraction: the fraction of slip in the robot
/// \param collision_radius: the radius of the robot for collision detection
/// \param max_range: the maximum range of the lidar sensor
/// \param basic_sensor_variance: the variance of the basic sensor
/// \param laser_min_range: the minimum range of the lidar sensor[m]
/// \param laser_max_range: the maximum range of the lidar sensor[m]
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
    declare_parameter("encoder_ticks_per_rev_sec", 651.89865);
    declare_parameter("draw_only", false);
    declare_parameter("input_noise", 0.0);
    declare_parameter("slip_fraction", 0.0);
    declare_parameter("collision_radius", 0.05);
    declare_parameter("max_range", 0.5);
    declare_parameter("basic_sensor_variance", 0.01);
    declare_parameter("laser_min_range", 0.11999999731779099);
    declare_parameter("laser_max_range", 3.5);

    rate = get_parameter("rate").as_int();
    draw_only_ = get_parameter("draw_only").as_bool();

    rclcpp::QoS qos_policy = rclcpp::QoS(rclcpp::KeepLast(10)).transient_local();
    rclcpp::QoS scan_qos_policy =
      rclcpp::QoS(rclcpp::KeepLast(20)).durability_volatile().best_effort();
    //publisher
    publisher_timestep_ = create_publisher<std_msgs::msg::UInt64>("/timestep", 10);
    publisher_walls_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/walls",
      qos_policy);
    publisher_obstacles_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/obstacles",
      qos_policy);
    if (!draw_only_) {
      publisher_fake_sensor_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/fake_sensor",
        qos_policy);
      pub_scan_ = create_publisher<sensor_msgs::msg::LaserScan>("/scan", scan_qos_policy);
      pub_sensor_ = create_publisher<nuturtlebot_msgs::msg::SensorData>("red/sensor_data", 10);
      pub_path_ = create_publisher<nav_msgs::msg::Path>("red/path", 10);

      //subscriber
      sub_wheel_ = create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
        "red/wheel_cmd", 10, std::bind(&Nusim::wheel_cmd_callback, this, std::placeholders::_1)
      );
      //service
      reset_service = create_service<std_srvs::srv::Empty>(
        "/reset", std::bind(
          &Nusim::reset_callback, this,
          std::placeholders::_1, std::placeholders::_2));

      teleport_service = create_service<nusim::srv::Teleport>(
        "/teleport", std::bind(
          &Nusim::teleport_callback, this,
          std::placeholders::_1, std::placeholders::_2));
    }

    //timer
    timer_ = create_wall_timer(
      1000ms / rate, std::bind(&Nusim::timer_callback, this));
    timer_sensor_ = create_wall_timer(
      200ms, std::bind(&Nusim::timer2_callback, this));
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
    encoder_ticks_per_rev_sec_ = get_parameter("encoder_ticks_per_rev_sec").as_double();
    x_ = x0_;
    y_ = y0_;
    theta_ = theta0_;
    arena_x_length_ = get_parameter("arena_x_length").as_double();
    arena_y_length_ = get_parameter("arena_y_length").as_double();
    obstacle_x_ = get_parameter("obstacles/x").as_double_array();
    obstacle_y_ = get_parameter("obstacles/y").as_double_array();
    std::vector<double> k = get_parameter("obstacles/x").as_double_array();
    obstacle_radius_ = get_parameter("obstacles/radius").as_double();
    input_noise = get_parameter("input_noise").as_double();
    slip_fraction = get_parameter("slip_fraction").as_double();
    collision_radius = get_parameter("collision_radius").as_double();
    max_range = get_parameter("max_range").as_double();
    basic_sensor_variance = get_parameter("basic_sensor_variance").as_double();
    laser_min_range = get_parameter("laser_min_range").as_double();
    laser_max_range = get_parameter("laser_max_range").as_double();
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

    auto new_left_wheel_joint = (msg->left_velocity * motor_cmd_per_rad_sec_ ) / rate;
    auto new_right_wheel_joint = (msg->right_velocity * motor_cmd_per_rad_sec_ ) / rate;
    std::normal_distribution<> d(0, input_noise);
    std::uniform_real_distribution<> u(-slip_fraction, slip_fraction);
    double slip_noise = u(get_random());

    // if (!turtlelib::almost_equal(new_left_wheel_joint, 0.0)) {
    //   new_left_wheel_joint += d(get_random());
    // }
    // if (!turtlelib::almost_equal(new_right_wheel_joint, 0.0)) {
    //   new_right_wheel_joint += d(get_random());
    // }

    //Save the wheel commands
    left_wheel_joint += (new_left_wheel_joint * encoder_ticks_per_rev_sec_ );
    right_wheel_joint += (new_right_wheel_joint * encoder_ticks_per_rev_sec_);
    //Change to sensor data

    sensor_data.left_encoder = left_wheel_joint;
    sensor_data.right_encoder = right_wheel_joint;


    //reverse the wheel commands to get the robot to move

    wheel_vels.phi_l = new_left_wheel_joint * (1 + slip_noise);
    wheel_vels.phi_r = new_right_wheel_joint * (1 + slip_noise);
    //DO FK
    robot_.forward_kinematics(wheel_vels);
    //Check for collision
    // Update robot position
    check_collision();
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

    robot_pose.header.frame_id = "nusim/world";
    robot_pose.header.stamp = get_clock()->now();
    robot_pose.pose.position.x = x;
    robot_pose.pose.position.y = y;
    robot_pose.pose.position.z = 0.0;
    robot_pose.pose.orientation.x = q.x();
    robot_pose.pose.orientation.y = q.y();
    robot_pose.pose.orientation.z = q.z();
    robot_pose.pose.orientation.w = q.w();
    red_path.poses.push_back(robot_pose);

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
    marker.color.a = 0.5;
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
      obstacle.color.a = 0.5;
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
        pose[1] = arena_y_length_ / 2 + wall_thickness / 2 + 0.01;
      }
      if (i == 2) {
        pose[1] *= -1;
      }
      if (i == 1 || i == 3) {
        scale[0] = wall_thickness;
        scale[1] = arena_y_length_;
        pose[0] = arena_x_length_ / 2 + wall_thickness / 2 + 0.01;
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
  /// \brief callback for the timer to publish the current time step
  void timer_callback()
  {
    auto message = std_msgs::msg::UInt64();
    message.data = count_++;
    publisher_timestep_->publish(message);

    if (!draw_only_) {
      //Publish the transform
      t.header.stamp = get_clock()->now();
      tf_broadcaster_->sendTransform(t);
      //Publish the sensor data
      sensor_data.stamp = get_clock()->now();
      pub_sensor_->publish(sensor_data);
      //Publish the path
      red_path.header.stamp = get_clock()->now();
      red_path.header.frame_id = "nusim/world";
      pub_path_->publish(red_path);
    }
  }
  /// \brief callback for the timer to publish the current time step
  void timer2_callback()
  {

    if (!draw_only_) {
      auto lidar_sensor = visualization_msgs::msg::MarkerArray();
      auto obstacle = visualization_msgs::msg::Marker();
      obstacle.header.frame_id = "red/base_footprint";
      obstacle.header.stamp = get_clock()->now();
      obstacle.type = visualization_msgs::msg::Marker::CYLINDER;

      for (int i = 0; i < int(obstacle_x_.size()); ++i) {
        obstacle.id = i;
        if (dist(i, max_range)) {
          std::normal_distribution<> d(0, basic_sensor_variance);
          auto obs_noise = d(get_random());
          obstacle.action = visualization_msgs::msg::Marker::ADD;
          obstacle.color.r = 1.0;
          obstacle.color.g = 1.0;
          obstacle.color.a = 1.0;
          obstacle.frame_locked = false;
          obstacle.scale.x = obstacle_radius_ * 2 + obs_noise;
          obstacle.scale.y = obstacle_radius_ * 2 + obs_noise;
          obstacle.scale.z = wall_height;
          const turtlelib::Transform2D Two(turtlelib::Vector2D{obstacle_x_[i], obstacle_y_[i]}, 0);
          const turtlelib::Vector2D sen_pos = (robot_.get_robot_pos().inv() * Two).translation();
          obstacle.pose.position.x = sen_pos.x + obs_noise;
          obstacle.pose.position.y = sen_pos.y + obs_noise;
          //  obstacle.pose.position.x = sen_pos.x ;
          // obstacle.pose.position.y = sen_pos.y;
          obstacle.pose.position.z = wall_height / 2;
        } else {
          obstacle.action = visualization_msgs::msg::Marker::DELETE;
        }
        lidar_sensor.markers.push_back(obstacle);
      }
      publisher_fake_sensor_->publish(lidar_sensor);

      scan();
      pub_scan_->publish(scan_data);
    }
  }
/// \brief Get the point at a given angle from the robot

  turtlelib::Vector2D get_point(double angle)
  {
    turtlelib::Vector2D p;
    p.x = laser_max_range * cos(angle);
    p.y = laser_max_range * sin(angle);
    return p;
    // return (robot_.get_robot_pos() * turtlelib::Transform2D(p, 0.0)).translation();
  }


/// \brief Get the distance to the obstacle
  double get_obs_dist(turtlelib::Vector2D p)
  {
    turtlelib::Vector2D pos =
      (robot_.get_robot_pos() * turtlelib::Transform2D(p, 0.0)).translation();
    double d = laser_max_range;
    double m = (pos.y - y_) / (pos.x - x_);
    double c = y_ - m * x_;

    // Check for walls
    // Wall 1
    if (pos.x < -arena_x_length_ / 2) {
      turtlelib::Vector2D p_C =
        turtlelib::Vector2D{-arena_x_length_ / 2, m * (-arena_x_length_ / 2) + c};
      turtlelib::Vector2D p_c_r =
        (robot_.get_robot_pos().inv() * turtlelib::Transform2D(p_C, 0.0)).translation();
      d = std::min(d, turtlelib::magnitude(p_c_r));
    }
    // Wall 2
    if (pos.x > arena_x_length_ / 2) {
      turtlelib::Vector2D p_C =
        turtlelib::Vector2D{arena_x_length_ / 2, m * (arena_x_length_ / 2) + c};
      turtlelib::Vector2D p_c_r =
        (robot_.get_robot_pos().inv() * turtlelib::Transform2D(p_C, 0.0)).translation();
      d = std::min(d, turtlelib::magnitude(p_c_r));
    }
    // Wall 3
    if (pos.y < -arena_y_length_ / 2) {
      turtlelib::Vector2D p_C =
        turtlelib::Vector2D{(-arena_y_length_ / 2 - c) / m, -arena_y_length_ / 2};
      turtlelib::Vector2D p_c_r =
        (robot_.get_robot_pos().inv() * turtlelib::Transform2D(p_C, 0.0)).translation();
      d = std::min(d, turtlelib::magnitude(p_c_r));
    }
    // Wall 4
    if (pos.y > arena_y_length_ / 2) {
      turtlelib::Vector2D p_C =
        turtlelib::Vector2D{(arena_y_length_ / 2 - c) / m, arena_y_length_ / 2};
      turtlelib::Vector2D p_c_r =
        (robot_.get_robot_pos().inv() * turtlelib::Transform2D(p_C, 0.0)).translation();
      d = std::min(d, turtlelib::magnitude(p_c_r));
    }
    // Check for obstacles
    for (int i = 0; i < int(obstacle_x_.size()); ++i) {
      // Checking if obstacle is in the range of the laser
      if (dist(i, laser_max_range)) {
        // If its in the range get perpendicular distance of center of the obstacle from the line
        double d_ = abs(m * obstacle_x_[i] - obstacle_y_[i] + c) / sqrt(m * m + 1);
        if (d_ <= obstacle_radius_) {
          // If perpendicular distance is less than radius and on the same side of the p, ,find intersection points
          double dis_r = sqrt(pow(x_ - obstacle_x_[i], 2) + pow(y_ - obstacle_y_[i], 2));
          double dis_p = sqrt(pow(pos.x - obstacle_x_[i], 2) + pow(pos.y - obstacle_y_[i], 2));
          // If the intersection points are in the range of the laser, find the distance to the obstacle
          if (turtlelib::almost_equal(
              sqrt(dis_r * dis_r - d_ * d_) + sqrt(dis_p * dis_p - d_ * d_),
              laser_max_range))
          {
            double intersect_dist = sqrt(dis_r * dis_r - d_ * d_) - sqrt(
              obstacle_radius_ * obstacle_radius_ - d_ * d_);
            d = std::min(d, intersect_dist);
          }
        }
      }
      if (d == laser_max_range || d < laser_min_range) {
        d = 0.0;
      }

    }
    return d;

  }

/// \brief Get the scan data
  void scan()
  {

    scan_data.ranges.clear();
    scan_data.header.frame_id = "red/base_scan";
    scan_data.header.stamp = get_clock()->now();
    scan_data.angle_min = 0;
    scan_data.angle_max = 2 * turtlelib::PI;
    scan_data.angle_increment = 0.01745329238474369;
    scan_data.time_increment = 0.0;
    scan_data.range_min = laser_min_range;
    scan_data.range_max = laser_max_range;
    scan_data.scan_time = 0.2;
    std::normal_distribution<> d(0, basic_sensor_variance);

    for (double i = scan_data.angle_min; i < scan_data.angle_max; i += scan_data.angle_increment) {
      turtlelib::Vector2D p = get_point(i);

      // double d = turtlelib::magnitude(p);
      scan_data.ranges.push_back(get_obs_dist(p) + d(get_random()));
    }
  }

  /// \brief Random number generator
  std::mt19937 & get_random()
  {
    // static variables inside a function are created once and persist for the remainder of the program
    static std::random_device rd{};
    static std::mt19937 mt{rd()};
    return mt;
  }

  /// \brief Distance between the robot and the obstacle
  bool dist(int i, double range)
  {
    double x = obstacle_x_[i];
    double y = obstacle_y_[i];
    auto r = robot_.get_robot_pos().translation();
    double d = sqrt(pow(x - r.x, 2) + pow(y - r.y, 2));

    if (range == 0.0 && d < obstacle_radius_ + collision_radius) {
      return true;
    } else if (d < range - obstacle_radius_ - collision_radius) {
      return true;
    }
    return false;
  }
  /// \brief Check for collision
  void check_collision()
  {
    for (int i = 0; i < int(obstacle_x_.size()); ++i) {
      if (dist(i, 0.0)) {
        robot_.set_pos(turtlelib::Transform2D(turtlelib::Vector2D{x_, y_}, theta_));
        return;
      }
    }
  }


  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service;
  rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport_service;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer_sensor_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher_timestep_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_walls_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_obstacles_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_fake_sensor_;
  rclcpp::Publisher<nuturtlebot_msgs::msg::SensorData>::SharedPtr pub_sensor_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_scan_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr sub_wheel_;
  geometry_msgs::msg::TransformStamped t = geometry_msgs::msg::TransformStamped();
  geometry_msgs::msg::PoseStamped robot_pose = geometry_msgs::msg::PoseStamped();

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
  bool draw_only_ = false;
  double input_noise = 0.0;
  double slip_fraction = 0.0;
  double collision_radius = 0.0;
  double max_range = 0.0;
  double basic_sensor_variance = 0.0;
  double laser_min_range = 0.0;
  double laser_max_range = 0.0;
  turtlelib::Diff_drive robot_;
  turtlelib::Wheel_state wheel_vels;
  nuturtlebot_msgs::msg::SensorData sensor_data;
  nav_msgs::msg::Path red_path;
  sensor_msgs::msg::LaserScan scan_data;

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
