/// \file
/// \brief Diffrential Drive Kinematics library.


#include<iosfwd> // contains forward definitions for iostream objects

#include"turtlelib/geometry2d.hpp"
#include"turtlelib/se2d.hpp"


namespace turtlelib
{   
    /// @brief wheel state
    /// \details This struct is used to store the wheel state.
    struct Wheel_state
    {
        /// \brief angle of the left wheel.
        double phi_l = 0.0;
        /// \brief angle of the right wheel.
        double phi_r = 0.0;
    };

    /// \brief Diffrential Drive Kinematics library.
    /// \details This class is used to calculate the forward kinematics of a diffrential drive robot.
    class Diff_drive
    {
    
    public:
        /// \brief Default constructor.
        /// \param wheel_track_ - distance between the wheels.
        /// \param wheel_radius_ - radius of the wheels.
        /// \param robot_pos_ - robot position, defaults to (0,0,0).
        Diff_drive(double wheel_track_, double wheel_radius_, Transform2D robot_pos_);

        /// \brief Default constructor.
        /// \param wheel_track_ - distance between the wheels.
        /// \param wheel_radius_ - radius of the wheels.
        Diff_drive(double wheel_track_, double wheel_radius_);

        /// \brief Forward kinematics of a diffrential drive robot.
        /// \param wheel_vels - the wheel velocities.
        void forward_kinematics(Wheel_state wheel_vels);
        
        /// \brief Inverse kinematics of a diffrential drive robot.
        /// \param twist - the twist to be applied.
        /// \returns the wheel velocities.
        Wheel_state inverse_kinematics(Twist2D twist);

        /// \brief Get the robot position.
        /// \returns the robot position.
        Transform2D get_robot_pos() const;

        /// \brief get wheel position.
        /// \returns the wheel position.
        Wheel_state get_wheel_pos() const;

        




    private:
        /// \brief Distance between the wheels.
        double wheel_track = 1.0;
        /// \brief Radius of the wheels.
        double wheel_radius = 0.5;
        /// @brief Angle of the wheels.
        Wheel_state wheel_state{0.0,0.0};
        /// \brief Robot position.
        Transform2D robot_pos = Transform2D();
    

    };
}
