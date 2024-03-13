/// \file ekf_slam.hpp
/// \brief EKF SLAM library.


#include<iosfwd> // contains forward definitions for iostream objects
#include<armadillo> //Matrix library
#include"turtlelib/geometry2d.hpp"
#include"turtlelib/se2d.hpp"
#include"turtlelib/diff_drive.hpp"

namespace turtlelib
{
    /// \brief EKF SLAM class.
    /// \details This class implements the Extended Kalman Filter for Simultaneous Localization and Mapping.
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
        int landmarks_observed = -1;
        int max_obs = 100;

     public:
        /// \brief Default constructor.
        /// \param max_obs - maximum number of observations.
        /// \details This constructor initializes the state vector and the covariance matrix.
        Ekf_slam(int max_obs);

        /// \brief Update state when obstacle is observed.
        /// \param Tmb - the transformation from the map to the base frame.
        /// \param x - x coordinate of the observed object.
        /// \param y - y coordinate of the observed object.
        /// \param id - id of the observed object.
        /// \returns void
        void object_observed(Transform2D Tmb, double x, double y, int id);

        /// \brief Prior update.
        /// \param state - the new state of the robot.
         /// \param T_del - the transformation from the previous state to the new state.
        void Prior_update(Transform2D state, Transform2D T_del);

        /// \brief Get the state.
        /// \returns the state vector.
        arma::Col<double> get_state();

        /// \brief Get the covariance matrix.
        /// \returns the covariance matrix.
        arma::Mat<double> get_sigma();  

         /// \brief Get the predicted observation.
         /// \returns the predicted observation.
        arma::Col<double> get_z_pred();

        /// \brief Compute the covariance matrix.
        /// \param T_del - the transformation from the previous state to the new state.
        /// \returns the covariance matrix.
        arma::Mat<double> compute_covariance(Transform2D T_del);

      /// \brief Get the observation id.
      /// \param Tmb - the transformation from the map to the base frame.
      /// \param x - x coordinate of the observed object.
      /// \param y - y coordinate of the observed object.
      /// \returns the id of the observed object.
      int data_association(Transform2D Tmb ,double x, double y);

        /// \brief Update the observation.
        /// \param id - the id of the observed object.
        /// \param x - the x coordinate of the observed object.
        /// \param y - the y coordinate of the observed object.
        void update_observation( double x, double y,int id);

        /// \brief Update the measurement model.
        void update_measurement_model();

        /// \brief Get polar coordinates of obstacles in map frame.
        /// \param mx - x coordinate of the observed object.
        /// \param my - y coordinate of the observed object.
        arma::Col<double> get_polar_coordinates(double mx, double my);

        /// \brief Update step for SLAM
        void posterior();

      

     };
}
