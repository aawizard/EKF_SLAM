/// \file slam.cpp
/// \brief Implements the slam node.
/// \brief Subscribes to odometry and fake sensor data and publishes the estimated observations.
/// \brief Publishes the path of the robot.
/// \brief Implemets EKF SLAM for a maximum number of obstacles.
///
///PARAMETERS:
///    \param body_id (string): body id of the robot
///    \param odom_id (string): odometry id of the robot
///PUBLISHES:
///    \param green/path (nav_msgs::msg::Path): path of the robot
///SUBSCRIBES:
///    \param odom (nav_msgs::msg::Odometry): odometry of the robot
///    \param fake_sensor (visualization_msgs::msg::MarkerArray): fake sensor data

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
#include <visualization_msgs/msg/marker_array.hpp>
#include "nuturtle_control/srv/init_config.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "turtlelib/ekf_slam.hpp"
#include "armadillo"

using namespace std::chrono_literals;
// using namespace turtlelib;

/// \brief Implemen.


/// \brief Implemets for a maximum number of obstacles
/// \param body_id - body id of the robot
/// \param odom_id - odometry id of the robot
class Slam : public rclcpp::Node
{
public:
  Slam()
  : Node("slam")
  {
    declare_parameter("body_id", "green/base_footprint");
    declare_parameter("odom_id", "green/odom");
    declare_parameter("use_fake_sensor", false);


    body_id = get_parameter("body_id").as_string();
    odom_id = get_parameter("odom_id").as_string();
    use_fake_sensor = get_parameter("use_fake_sensor").as_bool();

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&Slam::odom_callback, this, std::placeholders::_1));
    if(use_fake_sensor){
      // sub_fake_sensor_ = create_subscription<visualization_msgs::msg::MarkerArray>(
      //   "fake_sensor", 10, std::bind(&Slam::fake_sensor_callback, this, std::placeholders::_1));
    } else {
      sub_fake_sensor_ = create_subscription<visualization_msgs::msg::MarkerArray>(
        "estimate_landmark", 10, std::bind(&Slam::fake_sensor_callback, this, std::placeholders::_1));
    }
    pub_estimate_obs_ = create_publisher<visualization_msgs::msg::MarkerArray>("estimate_obs", 10);

    // pub_odom_ = create_publisher<nav_msgs::msg::Slam>("odom", 10);
    pub_path_ = create_publisher<nav_msgs::msg::Path>("green/path", 10);

    odom_robot_tf_.header.frame_id = odom_id;
    odom_robot_tf_.child_frame_id = body_id;
    odom_robot_tf_.header.stamp = get_clock()->now();
    map_odom_.header.frame_id = "map";
    map_odom_.child_frame_id = odom_id;
    map_odom_.header.stamp = get_clock()->now();
    
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {

    // Get the current robot pose and twist
    auto q = msg->pose.pose.orientation;
    double yaw = euler_from_quaternion_yaw(q);
    // m.getRPY(roll, pitch, yaw);
    turtlelib::Twist2D twist{msg->twist.twist.linear.x, 0, yaw};
    T_del *= turtlelib::integrate_twist(twist);
    // T_del = turtlelib::Transform2D(turtlelib::Vector2D{twist.x, twist.y}, twist.omega);
    Tob_ = turtlelib::Transform2D(
      turtlelib::Vector2D{msg->pose.pose.position.x,
        msg->pose.pose.position.y}, yaw);

    // Publish Transform
    odom_robot_tf_.header.stamp = get_clock()->now();
    odom_robot_tf_.transform.translation.x = Tob_.translation().x;
    odom_robot_tf_.transform.translation.y = Tob_.translation().y;
    odom_robot_tf_.transform.rotation = q;
    tf_broadcaster_->sendTransform(odom_robot_tf_);

    // Publish path
    robot_path_.header.stamp = get_clock()->now();
    robot_path_.header.frame_id = "map";
    geometry_msgs::msg::PoseStamped robot_pose_;
    robot_pose_.header.stamp = get_clock()->now();
    robot_pose_.header.frame_id = "map";
    robot_pose_.pose.position.x = Tmb_.translation().x;
    robot_pose_.pose.position.y = Tmb_.translation().y;
    robot_pose_.pose.orientation = q;
    robot_path_.poses.push_back(robot_pose_);
    pub_path_->publish(robot_path_);
  }

  void fake_sensor_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
  {
    
    // Implementing EKF SLAM, Assuming maximum of 30 landmarks
    Tmb_ = Tmo_ * Tob_;
    //EKF SLAM Step 1 - Prediction Step
    // WIll calculate A matrix and Q matrix
   
    T_del = turtlelib::Transform2D();

    //getting the state space model g(x,u,0)
    //Subscribing to fake sensor data
    for (int i = 0; i < static_cast<int>(msg->markers.size()); i++) {
      if (msg->markers[i].action == visualization_msgs::msg::Marker::ADD) {
        auto id = msg->markers[i].id;
        if(!use_fake_sensor){
          RCLCPP_INFO_STREAM(get_logger(), "\n\n obs  " << i);
          id = ekf.data_association(Tmb_, msg->markers[i].pose.position.x, msg->markers[i].pose.position.y);
          RCLCPP_INFO_STREAM(get_logger(), "ID: " << id);
        }
        if (id < max_obs){
        // Update the z_obs vector
        ekf.update_observation(
          msg->markers[i].pose.position.x, msg->markers[i].pose.position.y,
          id);
        // Update the state vector
        ekf.object_observed(
          Tmb_, msg->markers[i].pose.position.x, msg->markers[i].pose.position.y,
          id);
      }
      }
    }
  
    ekf.Prior_update(Tmb_, T_del);
    ekf.update_measurement_model();
    ekf.posterior();
    state_ = ekf.get_state();

    // Update Transform
    Tmb_ = turtlelib::Transform2D(turtlelib::Vector2D{state_[1], state_[2]}, state_[0]);
    Tmo_ = Tmb_ * Tob_.inv();
    // Publish map to odom transform
    map_odom_.header.stamp = get_clock()->now();
    map_odom_.transform.translation.x = Tmo_.translation().x;
    map_odom_.transform.translation.y = Tmo_.translation().y;
    tf2::Quaternion q;
    q.setRPY(0, 0, Tmo_.rotation());
    map_odom_.transform.rotation.x = q.x();
    map_odom_.transform.rotation.y = q.y();
    map_odom_.transform.rotation.z = q.z();
    map_odom_.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(map_odom_);

    publish_estimate_markers();
  }

/// \brief    Function to change quaternion to euler angles and get yaw
  double euler_from_quaternion_yaw(geometry_msgs::msg::Quaternion quaternion)
  {
    auto x = quaternion.x;
    auto y = quaternion.y;
    auto z = quaternion.z;
    auto w = quaternion.w;
    auto siny_cosp = 2 * (w * z + x * y);
    auto cosy_cosp = 1 - 2 * (y * y + z * z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  void publish_estimate_markers()
  {
    auto z_pred = ekf.get_z_pred();
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker marker;
    for (int i = 0; i < max_obs; i++) {
      marker.id = i;
      if (!turtlelib::almost_equal(
          state_[3 + 2 * i],
          0.0) || !turtlelib::almost_equal(state_[3 + 2 * i + 1], 0.0))
      {
        // if (!turtlelib::almost_equal(
        //     z_pred[2 * i],
        //     0.0) || !turtlelib::almost_equal(z_pred[2 * i + 1], 0.0))
        // {
        marker.header.frame_id = "map";
        // marker.header.frame_id = "green/base_footprint";
        marker.frame_locked = true;
        marker.header.stamp = get_clock()->now();

        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = state_[3 + 2 * i];
        marker.pose.position.y = state_[3 + 2 * i + 1];
        // auto xx = z_pred[2 * i] * cos(z_pred[2 * i + 1]);
        // auto yy = z_pred[2 * i] * sin(z_pred[2 * i + 1]);
        // marker.pose.position.x = xx;
        // marker.pose.position.y = yy;
        marker.pose.position.z = wall_height / 2;
        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = wall_height;
        marker.color.a = 1.0;
        marker.color.g = 1.0;
      } else {
        marker.action = visualization_msgs::msg::Marker::DELETE;
      }
      markers.markers.push_back(marker);
    }
    pub_estimate_obs_->publish(markers);
  }

  bool first = true;
  std::string body_id = "";
  std::string odom_id = "";
  int max_obs = 10;
  double wall_height = 0.3;
  bool use_fake_sensor = false;
  nav_msgs::msg::Odometry odom;
  geometry_msgs::msg::TransformStamped odom_robot_tf_;
  geometry_msgs::msg::TransformStamped map_odom_;
  nav_msgs::msg::Path robot_path_;
  turtlelib::Transform2D Tmo_;   // Transform from map to odom
  turtlelib::Transform2D Tmb_;   // Transform from map to base
  turtlelib::Transform2D Tob_;   // Transform from odom to base
  turtlelib::Transform2D T_del = turtlelib::Transform2D();  // Transform from odom to base in time 200ms
  turtlelib::Ekf_slam ekf = turtlelib::Ekf_slam(max_obs);


  arma::Col<double> state_ = arma::vec(3 + 2 * max_obs, arma::fill::zeros);  // State vector

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sub_fake_sensor_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_estimate_obs_;
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