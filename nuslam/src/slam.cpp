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
#include "turtlelib/diff_drive.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "armadillo"

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
  {    declare_parameter("body_id", "");
    declare_parameter("odom_id", "odom");
    declare_parameter("wheel_left", "");
    declare_parameter("wheel_right", "");
    declare_parameter("wheel_radius", -1.0);
    declare_parameter("track_width", -1.0);
    declare_parameter("input_noise", 0.001);

    body_id = get_parameter("body_id").as_string();
    odom_id = get_parameter("odom_id").as_string();
    wheel_left = get_parameter("wheel_left").as_string();
    wheel_right = get_parameter("wheel_right").as_string();
    wheel_radius = get_parameter("wheel_radius").as_double();
    track_width = get_parameter("track_width").as_double();
    input_noise = get_parameter("input_noise").as_double();

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
    sub_fake_sensor_ = create_subscription<visualization_msgs::msg::MarkerArray>(
      "fake_sensor", 10, std::bind(&Slam::fake_sensor_callback, this, std::placeholders::_1));
    pub_estimate_obs_ = create_publisher<visualization_msgs::msg::MarkerArray>("estimate_obs", 10);

    // pub_odom_ = create_publisher<nav_msgs::msg::Slam>("odom", 10);
    pub_path_ = create_publisher<nav_msgs::msg::Path>("green/path", 10);

    odom_robot_tf_.header.frame_id = odom_id;
    odom_robot_tf_.child_frame_id = body_id;
    odom_robot_tf_.header.stamp = this->get_clock()->now();
    map_odom_.header.frame_id = "map";
    map_odom_.child_frame_id = odom_id;
    map_odom_.header.stamp = this->get_clock()->now();
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
    Tob_ = turtlelib::Transform2D(turtlelib::Vector2D{msg->pose.pose.position.x, msg->pose.pose.position.y}, yaw);
    
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

  void fake_sensor_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
  {
    // Implementing EKF SLAM, Assuming maximum of 30 landmarks
    Tmb_ = Tmo_ * Tob_;
    //EKF SLAM Step 1 - Prediction Step
    //getting the state space model g(x,u,0)
    state_.subvec(0,2) = arma::Col<double>{Tmb_.rotation(), Tmb_.translation().x, Tmb_.translation().y};
     
    //Subscribing to fake sensor data
    for (int i = 0; i < static_cast<int>(msg->markers.size()); i++){
      turtlelib::Transform2D Tmobs = Tmb_ * turtlelib::Transform2D(turtlelib::Vector2D{msg->markers[i].pose.position.x, msg->markers[i].pose.position.y});
      auto r = sqrt(pow(msg->markers[i].pose.position.x,2) + pow(msg->markers[i].pose.position.y,2));
      auto theta = atan2(msg->markers[i].pose.position.y, msg->markers[i].pose.position.x);
      z_obs.subvec(2*i, 2*i+1) = {r, theta};
      // z_obs.subvec(2*(msg->markers[i].id), 2*(msg->markers[i].id)+1) = {Tmobs.translation().x, Tmobs.translation().y};
      // state_.subvec(3+2*(msg->markers[i].id), 3+2*(msg->markers[i].id)+1) = {Tmobs.translation().x, Tmobs.translation().y};

    }
    
    //Variance matrix
    set_prior_covariance();
    //Measurement Model
    update_measurement_vector();   // z and H are updated

    // Calculate Kalman Gain
    arma::Mat<double> R = arma::Mat<double>(2*max_obs, 2*max_obs, arma::fill::eye);
    // R.diag() = arma::randn(2*max_obs);
    std::normal_distribution<> d(0, input_noise);
    for(int i;i<2*max_obs;i++){
      R(i,i) = d(get_random());
    }
    // RCLCPP_INFO_STREAM(get_logger(), "\n "<<H);
    arma::Mat<double> K = Sigma * H.t() * arma::inv(H * Sigma * H.t() + R);
    // arma::Mat<double> K = Sigma * H.t() * arma::inv(H * Sigma * H.t());
    
    //EKF SLAM Step 2 - Update Step
    state_ += K * (z_obs - z_pred);
    Sigma = (arma::Mat<double>(3+2*max_obs, 3+2*max_obs, arma::fill::eye) - K * H) * Sigma;

    // Update Transform
    RCLCPP_INFO_STREAM(get_logger(), state_[1]<<" "<< state_[2] << " "<< state_[0] );
    Tmb_ = turtlelib::Transform2D(turtlelib::Vector2D{state_[1], state_[2]}, state_[0]);
    Tmo_ = Tmb_ * Tob_.inv();
    // Publish map to odom transform
    map_odom_.header.stamp = this->get_clock()->now();
    map_odom_.transform.translation.x = Tmo_.translation().x;
    map_odom_.transform.translation.y = Tmo_.translation().y;
    tf2::Quaternion q;
    q.setRPY(0, 0, Tmo_.rotation());
    map_odom_.transform.rotation.x = q.x();
    map_odom_.transform.rotation.y = q.y();
    map_odom_.transform.rotation.z = q.z();
    map_odom_.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(map_odom_);
    T_del = turtlelib::Transform2D();
    publish_estimate_markers();
  }
 
/// \brief    Function to change quaternion to euler angles and get yaw
  double euler_from_quaternion_yaw(geometry_msgs::msg::Quaternion quaternion){
    auto x = quaternion.x;
    auto y = quaternion.y;
    auto z = quaternion.z;
    auto w = quaternion.w;
    auto siny_cosp = 2 * (w * z + x * y);
    auto cosy_cosp = 1 - 2 * (y * y + z * z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  arma::Col<double> get_polar_coordinates(double mx, double my){
    double r = sqrt(pow(mx-state_.at(1),2) + pow(my-state_.at(2),2));
    double theta = turtlelib::normalize_angle(atan2(my-state_.at(2), mx-state_.at(1)) - state_.at(0));
    return {r, theta};
  }

  void set_prior_covariance(){
    // Calculating A matrix
    arma::Mat<double> A = arma::Mat<double>(3+2*max_obs, 3+2*max_obs, arma::fill::eye);
    if((T_del.translation().x != 0.0 || T_del.translation().y != 0.0) ){
      if(turtlelib::almost_equal(T_del.rotation(),0)){
        A(1,0) = -T_del.translation().x * sin(state_[0]);
        A(2,0)= T_del.translation().x * cos(state_[0]);
      }
      else{
        A(1,0) = (T_del.translation().x / T_del.rotation()) * (cos(turtlelib::normalize_angle(state_[0] + T_del.rotation())) - cos(state_[0]));
        A(2,0) = (T_del.translation().x / T_del.rotation()) * (sin(turtlelib::normalize_angle(state_[0] + T_del.rotation())) - sin(state_[0]));
      }
    }
    // Make Q matrix
    arma::Mat<double> Q = arma::join_cols(arma::join_rows(arma::eye(3,3), arma::Mat<double>(3, 2 * max_obs, arma::fill::zeros)),
                                      arma::Mat<double>(2 * max_obs, 3 + 2* max_obs, arma::fill::zeros));
    // Q.submat(0,0,2,2).diag() = arma::randn(3);    
    std::normal_distribution<> d(0, input_noise);
        Q.submat(0,0,2,2).diag() = arma::vec{d(get_random()), d(get_random()), d(get_random())};    
    Sigma = A * Sigma * A.t() + Q;
    // Sigma = A * Sigma * A.t();
    
  }

  void update_measurement_vector(){
    for (int i=0; i<max_obs;i++){
      if(state_[1] != 0.0 || state_[2] != 0.0){
        z_pred.subvec(2*i, 2*i +1) = get_polar_coordinates(state_[3+2*i], state_[3+2*i+1]);
        auto del_x = state_[3+2*i] - state_[1];
        auto del_y = state_[3+2*i+1] - state_[2];
        auto d = pow(del_x,2) + pow(del_y,2);
        H.submat(2*i, 0, 2*i+1, 2) = arma::join_rows(arma::vec{0,-1}, arma::vec{-del_x/sqrt(d), del_y/d}, arma::vec{-del_y/sqrt(d), -del_x/d}); 
        H.submat(2*i, 3 + 2*i, 2*i+1, 3 + 2*i+1) = arma::join_rows(arma::vec{del_x/sqrt(d), -del_y/d}, arma::vec{del_y/sqrt(d), del_x/d});
      }
    }    
  }

  void publish_estimate_markers(){
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker marker;
    for (int i=0; i<max_obs;i++){
      marker.id = i;
      if(state_[3+2*i] != 0.0 || state_[3+2*i+1] != 0.0){
        marker.header.frame_id = "map";
        marker.header.stamp = this->get_clock()->now();
        
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = state_[3+2*i];
        marker.pose.position.y = state_[3+2*i+1];
        marker.pose.position.z = wall_height/2;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = wall_height;
        marker.color.a = 1.0;
        marker.color.g = 1.0;
      }
      else{
        marker.action = visualization_msgs::msg::Marker::DELETE;
      }
      markers.markers.push_back(marker);
    }
    pub_estimate_obs_->publish(markers);
  }

   std::mt19937 & get_random()
  {
    // static variables inside a function are created once and persist for the remainder of the program
    static std::random_device rd{};
    static std::mt19937 mt{rd()};
    return mt;
  }
  

  
  std::string body_id = "";
  std::string odom_id = "";
  std::string wheel_left = "";
  std::string wheel_right = "";
  double wheel_radius = 0.0;
  double track_width = 0.0;
  double left_wheel_joint = 0.0;
  double right_wheel_joint = 0.0;
  double input_noise = 0.0;
  int max_obs = 30;
  double wall_height = 0.25;
  bool first = true;
  nav_msgs::msg::Odometry odom;
  geometry_msgs::msg::TransformStamped odom_robot_tf_;
  geometry_msgs::msg::TransformStamped map_odom_;
  nav_msgs::msg::Path robot_path_;
  turtlelib::Transform2D Tmo_;   // Transform from map to odom
  turtlelib::Transform2D Tmb_;   // Transform from map to base
  turtlelib::Transform2D Tob_;   // Transform from odom to base
  turtlelib::Transform2D T_del = turtlelib::Transform2D();  // Transform from odom to base in time 200ms 
  arma::Col<double> state_ = arma::vec(3+2*max_obs,arma::fill::zeros);  // State vector
  arma::Mat<double> Sigma = arma::join_cols(arma::join_rows(arma::Mat<double>(3, 3, arma::fill::zeros), arma::Mat<double>(3, 2 * max_obs, arma::fill::zeros)),
                                            arma::join_rows(arma::Mat<double>(2 * max_obs, 3, arma::fill::zeros), arma::Mat<double>(2 * max_obs, 2 * max_obs, arma::fill::eye) * 10000));  // Covariance matrix
  arma::Col<double> z_pred = arma::vec(2*max_obs,arma::fill::zeros);  // Measurement vector
  arma::Col<double> z_obs = arma::vec(2*max_obs,arma::fill::zeros);  // Measurement vector
  arma::Mat<double> H = arma::Mat<double>(2*max_obs, 3+2*max_obs, arma::fill::zeros);  // Jacobian matrix


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
