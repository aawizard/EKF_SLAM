#include"turtlelib/diff_drive.hpp"
#include"turtlelib/se2d.hpp"
#include"turtlelib/geometry2d.hpp"
#include<cmath>
#include <stdexcept>

namespace turtlelib
{

    Diff_drive::Diff_drive()
    {
        wheel_track = 0.16;
        wheel_radius = 0.033;
        robot_pos = Transform2D();
        wheel_state.phi_l = 0.0;
        wheel_state.phi_r = 0.0;
    }

    void Diff_drive::initialize(double wheel_track_, double wheel_radius_, Transform2D robot_pos_)
    {
        wheel_track = wheel_track_;
        wheel_radius = wheel_radius_;
        robot_pos = robot_pos_;
        wheel_state.phi_l = 0.0;
        wheel_state.phi_r = 0.0;        
    }

    Diff_drive::Diff_drive(double wheel_track_, double wheel_radius_, Transform2D robot_pos_)
    {
        wheel_track = wheel_track_;
        wheel_radius = wheel_radius_;
        robot_pos = robot_pos_;
        wheel_state.phi_l = 0.0;
        wheel_state.phi_r = 0.0;        
    }

    Diff_drive::Diff_drive(double wheel_track_, double wheel_radius_)
    {
        wheel_track = wheel_track_;
        wheel_radius = wheel_radius_;
        wheel_state.phi_l = 0.0;
        wheel_state.phi_r = 0.0;
    }

    void Diff_drive::set_pos(Transform2D robot_pos_)
    {
        robot_pos = robot_pos_;
    }

    Twist2D Diff_drive::Twist(Wheel_state wheel_vels)
    {
        //################# Start of citation 6 ##################
        // Equation 13.34 from Modern Robotics
        Twist2D twist;
        twist.omega = (wheel_radius/wheel_track)*(wheel_vels.phi_r - wheel_vels.phi_l);
        twist.x = (wheel_radius/2)*(wheel_vels.phi_r + wheel_vels.phi_l);
        twist.y = 0;
        return twist;
    }

    void Diff_drive::forward_kinematics(Wheel_state wheel_vels)
    {
        Twist2D twist = Twist(wheel_vels);
        Transform2D Tbb_ = integrate_twist(twist);
        robot_pos *= Tbb_;
        wheel_state.phi_r += wheel_vels.phi_r;
        wheel_state.phi_l += wheel_vels.phi_l;
    }
   // ################# End of citation 6 ##################

    void Diff_drive::forward_kinematics(Twist2D twist)
    {
        Transform2D Tbb_ = integrate_twist(twist);
        robot_pos *= Tbb_;
        Wheel_state wheel_vels = inverse_kinematics(twist);
        wheel_state.phi_r += wheel_vels.phi_r;
        wheel_state.phi_l += wheel_vels.phi_l;
    }

    Wheel_state Diff_drive::inverse_kinematics(Twist2D twist) 
    {
        if (almost_equal(twist.y,0)){
            Wheel_state wheel_vels;
            wheel_vels.phi_r = (1/wheel_radius)*(twist.x + (twist.omega*wheel_track/2));
            wheel_vels.phi_l = (1/wheel_radius)*(twist.x - (twist.omega*wheel_track/2));
            return wheel_vels;
        }
        else{
            throw std::logic_error("Twist cannot be accomplished without the wheels slipping.");
        }
    }

    Transform2D Diff_drive::get_robot_pos() const
    {
        return robot_pos;
    }

    Wheel_state Diff_drive::get_wheel_pos() const
    {
        return wheel_state;
    }

}