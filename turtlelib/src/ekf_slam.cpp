#include <armadillo>
#include <cmath>
#include <stdexcept>
#include <turtlelib/ekf_slam.hpp>
#include <cstdio>

namespace turtlelib
{
Ekf_slam::Ekf_slam(int maxobs)
{
  max_obs = maxobs;
  state_ = arma::zeros<arma::Col<double>>(3 + 2 * max_obs);
  z_pred = arma::zeros<arma::Col<double>>(2 * max_obs);
  z_obs = arma::zeros<arma::Col<double>>(2 * max_obs);
  H = arma::zeros<arma::Mat<double>>(2 * max_obs, 3 + 2 * max_obs);
  R = arma::Mat<double>(2 * max_obs, 2 * max_obs, arma::fill::eye) * 0.001;
  Q = arma::zeros<arma::Mat<double>>(3, 3);

  sigma_ = arma::join_cols(
    arma::Mat<double>(3, 3 + 2 * max_obs, arma::fill::zeros),
    arma::join_rows(
      arma::Mat<double>(2 * max_obs, 3, arma::fill::zeros),
      arma::Mat<double>(2 * max_obs, 2 * max_obs, arma::fill::eye) * 1e5));
}

void Ekf_slam::object_observed(Transform2D Tmb, double x, double y, int id)
{
  const auto Tmobs = Tmb * turtlelib::Transform2D(turtlelib::Vector2D{x, y}, 0.0);
  const auto x_ = Tmobs.translation().x;
  const auto y_ = Tmobs.translation().y;
  std::cerr << "object_observed : " << id << " x: " << x_ << " y: " << y << std::endl;
  if (state_(3 + 2 * id) == 0 && state_(3 + 2 * id + 1) == 0) {
    std::cerr<<"Updating id : " << id << " x: " << x_ << " y: " << y_ << std::endl;
    if (!almost_equal(sqrt(pow(x_, 2) + pow(y_, 2)), 0.0)) {
      state_(3 + 2 * id) = x_;
      state_(3 + 2 * id + 1) = y_;
    }
  }
}

arma::Mat<double> Ekf_slam::compute_covariance(Transform2D T_del)
{
  // T_del = T_del.inv();
  auto theta = normalize_angle(state_(0));
  auto del_theta = normalize_angle(T_del.rotation());
  auto del_x = T_del.translation().x;
  if (!almost_equal(T_del.rotation(), 0.0)) {
    del_x = (abs(T_del.translation().y) * del_theta) / (1 - cos(del_theta));
  }
  arma::Mat<double> A = arma::Mat<double>(3 + 2 * max_obs, 3 + 2 * max_obs, arma::fill::eye);
  if (almost_equal(del_theta, 0.0)) {
    A(1, 0) = -del_x * sin(theta);
    A(2, 0) = del_x * cos(theta);
  } else {
    A(1, 0) = (del_x / del_theta) * (-cos(theta) + cos(normalize_angle(theta + del_theta)));
    A(2, 0) = (del_x / del_theta) * (-sin(theta) + sin(normalize_angle(theta + del_theta)));
  }
  return A;
}


void Ekf_slam::Prior_update(Transform2D state, Transform2D T_del)
{

  auto A = compute_covariance(T_del);
  state_.subvec(
    0,
    2) = {normalize_angle(state.rotation()), state.translation().x, state.translation().y};
  arma::Mat<double> Q = arma::join_cols(
    arma::join_rows(arma::eye(3, 3), arma::Mat<double>(3, 2 * max_obs, arma::fill::zeros)),
    arma::Mat<double>(2 * max_obs, 3 + 2 * max_obs, arma::fill::zeros));
  Q.submat(0, 0, 2, 2).diag() = arma::vec{0.00001, 0.00001, 0.00001};
  sigma_ = A * sigma_ * A.t() + Q;
}

arma::Col<double> Ekf_slam::get_state()
{
  return state_;
}

arma::Mat<double> Ekf_slam::get_sigma()
{
  return sigma_;
}

void Ekf_slam::update_observation(double x, double y, int id)
{
  auto r = sqrt(pow(x, 2) + pow(y, 2));
  auto theta = normalize_angle(atan2(y, x));

  z_obs(2 * id) = r;
  z_obs(2 * id + 1) = theta;
}

arma::Col<double> Ekf_slam::get_polar_coordinates(double mx, double my)
{
  // if(almost_equal(mx,0) && almost_equal(my,0)){
  //     return {0,0};
  // }

  double r = sqrt(pow(mx - state_.at(1), 2) + pow(my - state_.at(2), 2));
  double theta = normalize_angle(
    normalize_angle(
      atan2(
        my - state_.at(2), mx - state_.at(
          1))) - normalize_angle(state_.at(0)));

  return {r, theta};
}

void Ekf_slam::update_measurement_model()
{
  H = arma::Mat<double>(2 * max_obs, 3 + 2 * max_obs, arma::fill::zeros);
  z_pred = arma::zeros<arma::Col<double>>(2 * max_obs);
  for (int i = 0; i < max_obs; i++) {
    if ((almost_equal(state_.at(3 + 2 * i), 0.0) && almost_equal(state_.at(3 + 2 * i + 1), 0.0)) || 
        (almost_equal(z_obs(2 * i), 0.0) && almost_equal(z_obs(2 * i + 1), 0.0))
    ) {
      continue;
    }
    z_pred.subvec(
      2 * i,
      2 * i + 1) = get_polar_coordinates(state_.at(3 + 2 * i), state_.at(3 + 2 * i + 1));
    auto del_x = state_.at(3 + 2 * i) - state_.at(1);
    auto del_y = state_.at(3 + 2 * i + 1) - state_.at(2);
    auto d = pow(del_x, 2) + pow(del_y, 2);

    H.submat(2 * i, 0, 2 * i + 1, 2) = arma::join_rows(
      arma::vec{0, -1}, arma::vec{-del_x / sqrt(
          d), del_y / d}, arma::vec{-del_y / sqrt(d), -del_x / d});
    H.submat(2 * i, 3 + 2 * i, 2 * i + 1, 4 + 2 * i) = arma::join_rows(
      arma::vec{del_x / sqrt(
          d), -del_y / d}, arma::vec{del_y / sqrt(d), del_x / d});

  }
}

void Ekf_slam::posterior()
{
  // R.diag() = arma::vec(2 * max_obs, arma::fill::ones) * 0.001;
  arma::Mat<double> S = (H * sigma_ * H.t()) + R;
  arma::Mat<double> K = sigma_ * H.t() * S.i();
  arma::Col<double> z_diff = z_obs - z_pred;
  for (int i = 0; i < max_obs; i++) {
    z_diff(2 * i + 1) = normalize_angle(z_diff(2 * i + 1));
  }
  state_ = state_ + (K * z_diff);
  state_.at(0) = normalize_angle(state_.at(0));
  sigma_ = (arma::eye(3 + 2 * max_obs, 3 + 2 * max_obs) - K * H) * sigma_;
  z_obs = arma::zeros<arma::Col<double>>(2 * max_obs);
}

arma::Col<double> Ekf_slam::get_z_pred()
{
  return z_pred;
}

int Ekf_slam::data_association(Transform2D Tmb ,double x, double y)
{
  std::cerr << "data_association" << std::endl;
  const auto Tmobs = Tmb * turtlelib::Transform2D(turtlelib::Vector2D{x, y}, 0.0);
  const auto x_ = Tmobs.translation().x;
  const auto y_ = Tmobs.translation().y;

  // z in robot frame
  auto r = sqrt(pow(x, 2) + pow(y, 2));
  auto theta = normalize_angle(atan2(y, x));

  int id = landmarks_observed + 1;
  auto mahanalobis_threshold = 45.0;
  auto R_k = arma::Mat<double>(2, 2, arma::fill::eye) * 0.5;

  //Step:2 defining z_pred_k ( zi = [r, theta])
  arma::Col<double> z_pred_k = arma::vec{r, theta};

  // psudo_state_ is the state vector with the new observation
  auto psudo_state_ = state_;
  psudo_state_.at(0) = normalize_angle(psudo_state_.at(0)); 
  // psudo_state_.subvec(3 + 2 * id, 3 + 2 * id + 1) = {x_, y_};

  std::cerr << "psudo_state_" << psudo_state_.t() << std::endl;
  //looping through the map find the best match
  for(auto i = 0; i <= landmarks_observed; i++){
    // if ((almost_equal(state_.at(3 + 2 * i), 0.0) && almost_equal(state_.at(3 + 2 * i + 1), 0.0))  ) {
    //   continue;
    // }

    arma::Col<double> z_p = get_polar_coordinates(psudo_state_.at(3 + 2 * i), psudo_state_.at(3 + 2 * i + 1));
    auto del_x = psudo_state_.at(3 + 2 * i) - psudo_state_.at(1);
    auto del_y = psudo_state_.at(3 + 2 * i + 1) - psudo_state_.at(2);
    auto d = pow(del_x, 2) + pow(del_y, 2);
    arma::Mat<double> H_k = arma::Mat<double>(2, 3 + 2 * max_obs, arma::fill::zeros);

    H_k.submat(0, 0, 1, 2) = arma::join_rows(
      arma::vec{0, -1}, arma::vec{-del_x / sqrt(
          d), del_y / d}, arma::vec{-del_y / sqrt(d), -del_x / d});

    H_k.submat(0, 3 + 2 * i, 1, 4 + 2 * i) = arma::join_rows(
      arma::vec{del_x / sqrt(
          d), -del_y / d}, arma::vec{del_y / sqrt(d), del_x / d});

    arma::Mat<double> cov_si = (H_k * sigma_ * H_k.t()) + R_k;
    std::cerr << "cov_si" << cov_si << std::endl;
    // std::cerr << "H_k" << H_k << std::endl;
    // std::cerr << "sigma_" << sigma_ << std::endl;
    arma::Col<double> z_diff = z_pred_k - z_p;
    z_diff.at(1) = normalize_angle(normalize_angle(z_pred_k.at(1))- normalize_angle(z_p.at(1)));
    auto a = z_diff(0);
    auto b = z_diff(1);
    double mahalanobis = as_scalar(z_diff.t() * cov_si.i() * z_diff);
    if (cov_si.at(0,0) > 1e5) {
      // mahalanobis = as_scalar(z_diff.t() * cov_si * z_diff);
      mahanalobis_threshold = 1e-6;
    }
    
    // id = z_diff.at(0) * 100;
   std::cerr<<"Distance : " << mahalanobis << " obs: " << i <<" z_diff"<< z_diff.t()<<std::endl;
    if(mahalanobis < mahanalobis_threshold){
      id = i;
      mahanalobis_threshold = mahalanobis;
    }
  }

  if (id > landmarks_observed) {
    landmarks_observed++;
    }
  return id;
}
}  // namespace turtlelib

