#include <cmath>
#include <stdexcept>
#include <armadillo> //Matrix library
#include <turtlelib/landmark_calc.hpp>

namespace turtlelib
{
Landmarklib::Landmarklib(
  std::vector<double> sensor_reading_, double angle_min_,
  double angle_increment_)
{
  sensor_reading = sensor_reading_;
  this->angle_min = angle_min_;
  this->angle_increment = angle_increment_;
}
Landmarklib::Landmarklib(std::vector<std::vector<Vector2D>> clusters_)
{
  clusters = clusters_;
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

void Landmarklib::cluster_laser_data()
{
  double threshold = 0.20;
  Vector2D last_point = get_cartesian_coordinates(sensor_reading.at(0), angle_min);
  clusters = {};
  std::vector<Vector2D> cluster;
  auto theta = angle_min;
  // Iterate over sensor_reading
  for (auto i = 0; i < static_cast<int>(sensor_reading.size()); i++) {
    theta = angle_min + (i * angle_increment);
    auto point = get_cartesian_coordinates(sensor_reading.at(i), theta);

    if (check_threshold(last_point, point, threshold)) {
      cluster.push_back(point);
    } else {
      clusters.push_back(cluster);
      cluster = {point};
    }
    last_point = point;
  }
  //Wrap up the last cluster
  auto fisrt_cluster_point = clusters.at(0).at(0);
  auto last_cluster_point = clusters.at(clusters.size() - 1).at(
    clusters.at(
      clusters.size() - 1).size() - 1);
  if (check_threshold(fisrt_cluster_point, last_cluster_point, threshold)) {
    clusters.at(0).insert(
      clusters.at(0).end(), clusters.at(
        clusters.size() - 1).begin(), clusters.at(clusters.size() - 1).end());
    clusters.pop_back();
  }


  // Return clusters with less then 2 points
  for (auto it = clusters.begin(); it != clusters.end(); ) {
    if (it->size() < 4) {
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

std::vector<std::vector<double>> Landmarklib::get_landmark_centroids()
{
  for (auto cluster : clusters) {
    std::vector<double> centroid = circle_fitting(cluster);
    // if(centroid.at(2) < 0.1){

    landmark_centroids.push_back(centroid);
    // }
  }
  return landmark_centroids;
}


std::vector<double> Landmarklib::circle_fitting(std::vector<Vector2D> cluster)
{
  //Step1: Mean of the x and y of the cluster
  Vector2D sum = {0, 0};
  for (auto point : cluster) {
    sum += point;
  }
  double x_mean = sum.x / cluster.size();
  double y_mean = sum.y / cluster.size();

  //Step2: shift the points so that centroid is at origin
  std::vector<Vector2D> shifted_points;
  for (auto & point : cluster) {
    shifted_points.push_back(point - Vector2D{x_mean, y_mean});

  }
  //Step3: compute z
  std::vector<double> z;
  for (auto point : cluster) {
    z.push_back(point.x * point.x + point.y * point.y);
  }
  //Step4: compute the mean of z
  double z_mean = 0;
  for (auto z_value : z) {
    z_mean += z_value;
  }
  z_mean = z_mean / cluster.size();

  //Step5: Data matrix from n data points
  arma::mat Z(cluster.size(), 4);
  for (auto i = 0; i < static_cast<int>(cluster.size()); i++) {
    Z(i, 0) = z.at(i);
    Z(i, 1) = cluster.at(i).x;
    Z(i, 2) = cluster.at(i).y;
    Z(i, 3) = 1;
  }

  //Step6: Compute the matrix M
  arma::mat M = (1 / cluster.size()) * (Z.t() * Z);

  //Step7: Compute the matrix H (Hyperaccurate algebraic fit)
  arma::mat H = arma::Mat<double>(4, 4, arma::fill::eye);
  H(0, 0) = 8 * z_mean;
  H(3, 3) = 0;
  H(0, 3) = 2;
  H(3, 0) = 2;

  //Step8: Compute the matrix H inverse
  arma::mat H_inv = arma::Mat<double>(4, 4, arma::fill::eye);
  H_inv(0, 0) = 0;
  H_inv(3, 3) = -2 * z_mean;
  H_inv(0, 3) = 0.5;
  H_inv(3, 0) = 0.5;

  //Step9: Compute singular value decomposition of Z
  arma::mat U;
  arma::vec s;
  arma::mat V;
  arma::svd(U, s, V, Z);

  //Step10: Compute the matrix A
  arma::vec A;
  double min_s = s.at(0);
  for (int i = 0; i < 4; i++) {
    if (s.at(i) < min_s) {
      min_s = s.at(i);
    }
  }
  if (min_s < 1e-12) {
    A = V.at(3);
  }
  // //Step11: Compute the matrix A
  else {
    arma::mat Y = V * arma::diagmat(s) * V.t();
    arma::mat Q = Y * H_inv * Y;
    arma::vec eigval;
    arma::mat eigvec;
    arma::eig_sym(eigval, eigvec, Q);
    arma::mat A1;
    double min = 1e12;
    for (int i = 0; i < 4; i++) {
      if (eigval.at(i) < min && eigval.at(i) > 0) {
        min = eigval.at(i);
        A1 = eigvec.col(i);
      }
    }
    A = Y.i() * A1;
    //
  }

  //Step12: Compute the matrix a,b,R
  double a = -A.at(1) / (2 * A.at(0));
  double b = -A.at(2) / (2 * A.at(0));
  double R =
    sqrt(
    (A.at(1) * A.at(1) + A.at(2) * A.at(2) - 4 * A.at(0) * A.at(3)) /
    (4 * A.at(0) * A.at(0)));
  return {a + x_mean, b + y_mean, R};
}

}  // namespace turtlelib
