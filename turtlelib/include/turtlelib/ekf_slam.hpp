/// \file
/// \brief EKF SLAM library.


#include<iosfwd> // contains forward definitions for iostream objects
#include<armadillo> //Matrix library
#include"turtlelib/geometry2d.hpp"
#include"turtlelib/se2d.hpp"
#include"turtlelib/diff_drive.hpp"

namespace turtlelib
{
     class Ekf_slam
     {
     private:
        arma::Col<double> state_;
        arma::Mat<double> sigma_;
        arma::Col<double> z_pred;
        arma::Col<double> z_obs;
        arma::Mat<double> H;
        arma::Mat<double> R;
        arma::Mat<double> Q;
        int max_obs = 100;

     public:
        /// \brief Default constructor.
        /// \param max_obs - maximum number of observations.
        /// \details This constructor initializes the state vector and the covariance matrix.
        Ekf_slam(int max_obs);

        /// \brief Update state when obstacle is observed.
        /// \param x - x coordinate of the observed object.
        /// \param y - y coordinate of the observed object.
        /// \param id - id of the observed object.
        /// \returns void
        void object_observed(double x, double y, int id);

        /// \brief Prior update.
        /// \param state - the new state of the robot.
        void Prior_update(Transform2D state, Transform2D T_del);

        /// \brief Get the state.
        /// \returns the state vector.
        arma::Col<double> get_state();

        /// \brief Get the covariance matrix.
        /// \returns the covariance matrix.
        arma::Mat<double> get_sigma();  

        /// \brief Compute the covariance matrix.
        /// \returns the covariance matrix.
        arma::Mat<double> compute_covariance(Transform2D T_del);
     };
}
