#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"


using namespace std::chrono_literals;


/// \brief TODO: Fill in the documentation for this class
/// \param wheel_radius - radius of the wheel [m]
/// \param track_width - distance between the wheels [m]
/// \param motor_cmd_max - maximum motor command [rad/s]
/// \param motor_cmd_per_rad_sec - motor command per rad/s
/// \param encoder_ticks_per_rev - encoder ticks per revolution
/// \param collision_radius - collision radius of the robot [m]
class Turtle_control : public rclcpp::Node
{
public:
  Turtle_control()
  : Node("turtle_control")
  {
    declare_parameter("wheel_radius", 0.033);
    declare_parameter("track_width", 0.160);
    declare_parameter("motor_cmd_max", 265);
    declare_parameter("motor_cmd_per_rad_sec", 0.024);
    declare_parameter("encoder_ticks_per_rev", 4096);
    declare_parameter("collision_radius", 0.11);
  }

};

/// @brief    main function for the turtle_control node
/// @param argc
/// @param argv
/// @return
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Turtle_control>());
  rclcpp::shutdown();
  return 0;
}
