#include <cmath>
#include <stdexcept>

#include <turtlelib/landmark_calc.hpp>

namespace turtlelib
{
Landmarklib::Landmarklib(std::vector<double> sensor_reading_, double angle_min_, double angle_increment_)
{
  sensor_reading = sensor_reading_;
  this->angle_min = angle_min_;
  this->angle_increment = angle_increment_;
}   

Vector2D Landmarklib::get_cartesian_coordinates(double r, double theta)
{
  double x = r * cos(theta);
  double y = r * sin(theta);
  return Vector2D{x, y};
}

bool check_threshold(Vector2D point1, Vector2D point2, double threshold)
{
  point2 = point2 - point1;
  auto distance = magnitude(point2);
  return distance < threshold;
}

void Landmarklib::cluster_laser_data() {
    double threshold = 0.20;
    Vector2D last_point = get_cartesian_coordinates(sensor_reading.at(0), angle_min);
    clusters = {};
    std::vector<Vector2D> cluster;
    auto theta = angle_min;
    // Iterate over sensor_reading
    for (int i = 0; i < sensor_reading.size(); i++) {
      theta = angle_min + (i * angle_increment);
      auto point = get_cartesian_coordinates(sensor_reading.at(i), theta);
      
      if(check_threshold(last_point, point, threshold))
      {
        cluster.push_back(point);
      }
      else
      {
        clusters.push_back(cluster);
        cluster = {point};
      }
      last_point = point;
    }
  // Return clusters with less then 2 points
for(auto it = clusters.begin(); it != clusters.end(); ) {
    if(it->size() < 4) {
        it = clusters.erase(it);
    } else {
        ++it;
    }
}
}

std::vector<std::vector<Vector2D>> Landmarklib::get_clusters()
{
  return clusters;
}

}  // namespace turtlelib