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

#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <rclcpp/qos.hpp>

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
    //parameter for wall
    this->declare_parameter("theta0", 0.0);
    this->declare_parameter("arena_x_length", 2.0);
    this->declare_parameter("arena_y_length", 2.0);

    //parameters for obstacles
    this->declare_parameter("obstacles/x", obstacle_x_);
    this->declare_parameter("obstacles/y", obstacle_y_);
    this->declare_parameter("obstacles/radius", 0.05);

    auto rate = get_parameter("rate").as_int();
    rclcpp::QoS qos_policy = rclcpp::QoS(rclcpp::KeepLast(10)).transient_local();

    publisher_timestep_ = this->create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
    publisher_walls_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/walls",
      qos_policy);
    publisher_obstacles_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/obstacles",
      qos_policy);
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

    x_ = get_parameter("x0").as_double();
    y_ = get_parameter("y0").as_double();
    theta_ = get_parameter("theta0").as_double();
    arena_x_length_ = get_parameter("arena_x_length").as_double();
    arena_y_length_ = get_parameter("arena_y_length").as_double();
    obstacle_x_ = get_parameter("obstacles/x").as_double_array();
    obstacle_y_ = get_parameter("obstacles/y").as_double_array();
    std::vector<double> k = get_parameter("obstacles/x").as_double_array();
    obstacle_radius_ = get_parameter("obstacles/radius").as_double();
    change_position(x_, y_, theta_);

    show_walls();
    make_obstacles();
  }

private:
  void reset_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    count_ = 0;
    change_position(0.0, 0.0, 0.0);
    RCLCPP_INFO(get_logger(), "Resetting count");
  }

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

  void timer_callback()
  {
    auto message = std_msgs::msg::UInt64();
    message.data = count_++;
    // RCLCPP_INFO_STREAM(get_logger(), "Publishing: '" << message.data << "'");
    publisher_timestep_->publish(message);
    t.header.stamp = get_clock()->now();
    tf_broadcaster_->sendTransform(t);
  }

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
  geometry_msgs::msg::TransformStamped t = geometry_msgs::msg::TransformStamped();
  double x_ = 0.0;
  double y_ = 0.0;
  double theta_ = 0.0;
  double wall_height = 0.25;
  double wall_thickness = 0.10;
  double arena_x_length_ = 1.0;
  double arena_y_length_ = 1.0;

  std::vector<double> obstacle_x_ = {0.25, 0.35};
  std::vector<double> obstacle_y_ = {0.25, -0.25};
  float obstacle_radius_ = 0.05;
  size_t count_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nusim>());
  rclcpp::shutdown();
  return 0;
}
