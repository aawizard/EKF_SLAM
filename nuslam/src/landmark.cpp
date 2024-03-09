/// \file landmark.cpp
/// \brief Subscribes to joint states and publishes odometry of the robot
///
///PARAMETERS:
///    
///PUBLISHES:
///    \param estimate_landmark (visualization_msgs::msg::MarkerArray): Marker_array for the estimated landmarks
///SUBSCRIBES:
///    \param scan (sensor_msgs::msg::LaserScan): Laser scan data

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "armadillo"
#include "turtlelib/landmark_calc.hpp"

using namespace std::chrono_literals;
// using namespace turtlelib;

/// \brief subscribes to joint states and publishes odometry of the robot
class Landmark : public rclcpp::Node
{
public:
  Landmark()
  : Node("landmark")
  {
    rclcpp::QoS scan_qos_policy =
      rclcpp::QoS(rclcpp::KeepLast(20)).durability_volatile().best_effort();
    sub_laser = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", scan_qos_policy, std::bind(&Landmark::laser_callback, this, std::placeholders::_1));
    pub_estimate_landmark_ = create_publisher<visualization_msgs::msg::MarkerArray>("estimate_landmark", 10);
  }

private:

  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
      std::vector<double> ranges_vector(msg->ranges.begin(), msg->ranges.end());
      turtlelib::Landmarklib landmarklib = turtlelib::Landmarklib(ranges_vector, msg->angle_min, msg->angle_increment);
      landmarklib.cluster_laser_data();
      auto clusters = landmarklib.get_clusters();
      publish_clusters(clusters);

  }

  /// \brief Publish clusters as markers
   void publish_clusters(const std::vector<std::vector<turtlelib::Vector2D>>& clusters) {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker marker;
    auto reading = 0;
    auto cluster_count = 1;
    for (const auto& cluster : clusters) {
        for (const auto& point : cluster) {
        marker.id = reading;
        marker.header.frame_id = "red/base_scan";
        marker.header.stamp = get_clock()->now();
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = point.x;
        marker.pose.position.y = point.y;
        marker.pose.position.z = 0.0;
        marker.scale.x = 0.01;
        marker.scale.y = 0.01;
        marker.scale.z = 0.01;
        marker.color.a = 1.0;
        // marker.color.g = 1.0;
        if(cluster_count % 3 == 0)
        {
          marker.color.r = 1.0;
          marker.color.g = 0.0;
          marker.color.b = 0.0;
        }
        else if(cluster_count % 3 == 1)
        {
          marker.color.r = 0.0;
          marker.color.g = 0.0;
          marker.color.b = 1.0;
        }
        else
        {
          marker.color.r = 0.0;
          marker.color.g = 1.0;
          marker.color.b = 0.0;
        }
        markers.markers.push_back(marker);
        reading++;
      }
      cluster_count++;
    }
    pub_estimate_landmark_->publish(markers);
   }

//   void publish_estimate_markers()
//   {
//     visualization_msgs::msg::MarkerArray markers;
//     visualization_msgs::msg::Marker marker;
//     for (int i = 0; i < max_obs; i++) {
//       marker.id = i;
//       if (!turtlelib::almost_equal(
//           state_[3 + 2 * i],
//           0.0) || !turtlelib::almost_equal(state_[3 + 2 * i + 1], 0.0))
//       {
//         marker.header.frame_id = "map";
//         // marker.header.frame_id = "green/base_footprint";
//         marker.frame_locked = true;
//         marker.header.stamp = get_clock()->now();

//         marker.type = visualization_msgs::msg::Marker::CYLINDER;
//         marker.action = visualization_msgs::msg::Marker::ADD;
//         marker.pose.position.x = state_[3 + 2 * i];
//         marker.pose.position.y = state_[3 + 2 * i + 1];
//         marker.pose.position.z = wall_height / 2;
//         marker.scale.x = 0.05;
//         marker.scale.y = 0.05;
//         marker.scale.z = wall_height;
//         marker.color.a = 0.5;
//         marker.color.g = 1.0;
//       } else {
//         marker.action = visualization_msgs::msg::Marker::DELETE;
//       }
//       markers.markers.push_back(marker);
//     }
//     pub_estimate_obs_->publish(markers);
//   }



  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laser;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_estimate_landmark_;

};

/// @brief    main function for the Landmark node
/// @param argc
/// @param argv
/// @return
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Landmark>());
  rclcpp::shutdown();
  return 0;
}
