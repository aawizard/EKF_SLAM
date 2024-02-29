#include<armadillo>
#include<cmath>
#include <stdexcept>
#include <turtlelib/ekf_slam.hpp>

namespace turtlelib
{
    Ekf_slam::Ekf_slam(int maxobs)
    {   
        max_obs = maxobs;
        state_ = arma::zeros<arma::Col<double>>(3 + 2*max_obs);
        z_pred = arma::zeros<arma::Col<double>>(2*max_obs);
        z_obs = arma::zeros<arma::Col<double>>(2*max_obs);
        H = arma::zeros<arma::Mat<double>>(2*max_obs, 3 + 2*max_obs);
        R = arma::zeros<arma::Mat<double>>(2*max_obs, 2*max_obs);
        Q = arma::zeros<arma::Mat<double>>(3, 3);

        sigma_ = arma::join_cols(arma::join_rows(arma::Mat<double>(3, 3, arma::fill::zeros), arma::Mat<double>(3, 2 * max_obs, arma::fill::zeros)),
                                            arma::join_rows(arma::Mat<double>(2 * max_obs, 3, arma::fill::zeros), arma::Mat<double>(2 * max_obs, 2 * max_obs, arma::fill::eye) * 10000));
    }

    void Ekf_slam::object_observed(double x, double y, int id){
        if (state_(3 + 2*id) == 0 && state_(3 + 2*id + 1) == 0){
            if(!almost_equal(x, 0.0, .0001) || !almost_equal(y, 0.0, 0.0001)){
              state_(3 + 2*id) = x;
            state_(3 + 2*id + 1) = y;
            }
        }
    }
    
    arma::Mat<double> Ekf_slam::compute_covariance(Transform2D T_del){
        T_del = T_del.inv();
        auto theta = state_(0);
        auto del_theta = T_del.rotation();
        auto del_x = T_del.translation().x;
        if (!almost_equal(T_del.rotation(),0.0)){
            del_x = (abs(T_del.translation().y)*del_theta)/(1-cos(del_theta));
        }
        arma::Mat<double> A = arma::Mat<double>(3+2*max_obs, 3+2*max_obs, arma::fill::eye);
        if(almost_equal(del_theta, 0.0)){
            A(1,0) = -del_x*sin(theta);
            A(2,0) = del_x*cos(theta);
        }
        else{
            A(1,0) = (del_x/del_theta) * (-cos(theta) + cos(theta + del_theta));
            A(2,0) = (del_x/del_theta) * (-sin(theta) + sin(theta + del_theta));
        }
        return A;
    }


    void Ekf_slam::Prior_update(Transform2D state, Transform2D T_del){
        state_.subvec(0, 2) = {state.rotation(),state.translation().x, state.translation().y};
        auto A = compute_covariance(T_del);
        
        arma::Mat<double> Q = arma::join_cols(arma::join_rows(arma::eye(3,3), arma::Mat<double>(3, 2 * max_obs, arma::fill::zeros)),
                                      arma::Mat<double>(2 * max_obs, 3 + 2* max_obs, arma::fill::zeros));
        Q.submat(0,0,2,2).diag() = arma::randn(3);    
        // std::normal_distribution<> d(0, 0.001);
        //     Q.submat(0,0,2,2).diag() = arma::vec{d(get_random()), d(get_random()), d(get_random())};    
        sigma_ = A * sigma_ * A.t() + Q;
    }

    arma::Col<double> Ekf_slam::get_state(){
        return state_;
    }

    arma::Mat<double> Ekf_slam::get_sigma(){
        return sigma_;
    }
}