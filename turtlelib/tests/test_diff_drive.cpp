#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include"turtlelib/geometry2d.hpp"
#include"turtlelib/se2d.hpp"
#include"turtlelib/diff_drive.hpp"
#include<sstream>
#include<cmath>
// #include <catch2/catch.hpp>


using namespace turtlelib;

TEST_CASE("Test robot_state","[get_robot_pos]"){
    Transform2D tf({1.0,2.0}, PI/2);
    Diff_drive dd(1.0, 1.0, tf);
    REQUIRE_THAT(dd.get_robot_pos().rotation(),
        Catch::Matchers::WithinAbs(PI/2,0.1));
    REQUIRE_THAT(dd.get_robot_pos().translation().x,
        Catch::Matchers::WithinAbs(1.0,0.1));
    REQUIRE_THAT(dd.get_robot_pos().translation().y,
        Catch::Matchers::WithinAbs(2.0,0.1));
}

TEST_CASE("Test wheel_state","[get_wheel_pos]"){
    Transform2D tf({1.0,2.0}, PI/2);
    Diff_drive dd(1.0, 1.0, tf);
    REQUIRE_THAT(dd.get_wheel_pos().phi_l,
        Catch::Matchers::WithinAbs(0.0,0.1));
    REQUIRE_THAT(dd.get_wheel_pos().phi_r,
        Catch::Matchers::WithinAbs(0.0,0.1));
}

// TEST_CASE("Test forward_kinematics", "[forward_kinematics]"){
//     Transform2D tf({1.0,2.0}, 0.0);
//     Diff_drive dd(1.0, 0.5, tf);

//     // Test: same wheel speeds - drives forward
//     Wheel_state wheel_vel{0.2, 0.2};

//     dd.forward_kinematics(wheel_vel);
//     REQUIRE_THAT(dd.get_robot_pos().rotation(),
//         Catch::Matchers::WithinAbs(0.0,0.1));
//     REQUIRE_THAT(dd.get_robot_pos().translation().x,
//         Catch::Matchers::WithinAbs(1.1,0.01));
//     REQUIRE_THAT(dd.get_robot_pos().translation().y,
//         Catch::Matchers::WithinAbs(2.0,0.01));
//     REQUIRE_THAT(dd.get_wheel_pos().phi_l,
//         Catch::Matchers::WithinAbs(0.2,0.01));
//     REQUIRE_THAT(dd.get_wheel_pos().phi_r,
//         Catch::Matchers::WithinAbs(0.2,0.01));

//     // Test: different wheel speeds
//     Wheel_state wheel_vel1{0.2, -0.1};
//     dd.forward_kinematics(wheel_vel1);
//     REQUIRE_THAT(dd.get_robot_pos().rotation(),
//         Catch::Matchers::WithinAbs(-0.15,0.1));
//     REQUIRE_THAT(dd.get_robot_pos().translation().x,
//         Catch::Matchers::WithinAbs(1.1,0.1));
//     REQUIRE_THAT(dd.get_robot_pos().translation().y,
//         Catch::Matchers::WithinAbs(2.0,0.01));
//     REQUIRE_THAT(dd.get_wheel_pos().phi_l,
//         Catch::Matchers::WithinAbs(0.4,0.01));
//     REQUIRE_THAT(dd.get_wheel_pos().phi_r,
//         Catch::Matchers::WithinAbs(0.1,0.01));


//     // Test: pure rotation
//     Wheel_state wheel_vel2{-0.1, 0.1};
//     dd.forward_kinematics(wheel_vel2);
//     REQUIRE_THAT(dd.get_robot_pos().rotation(),
//         Catch::Matchers::WithinAbs(-0.05,0.1));
//     REQUIRE_THAT(dd.get_robot_pos().translation().x,
//         Catch::Matchers::WithinAbs(1.1,0.1));
//     REQUIRE_THAT(dd.get_robot_pos().translation().y,
//         Catch::Matchers::WithinAbs(2.0,0.01));
//     REQUIRE_THAT(dd.get_wheel_pos().phi_l,
//         Catch::Matchers::WithinAbs(0.3,0.01));
//     REQUIRE_THAT(dd.get_wheel_pos().phi_r,
//         Catch::Matchers::WithinAbs(0.2,0.01));
    
// }

TEST_CASE("Test Inverse Kinematics","[inverse_kinamatics]"){
    Transform2D tf({1.0,2.0}, 0.0);
    Diff_drive dd(1.0, 1.0, tf);

    // Test: same wheel speeds - drives forward
    Twist2D twist{0.0, 0.2, 0.0};
    Wheel_state wheel_vel = dd.inverse_kinematics(twist);
    REQUIRE_THAT(wheel_vel.phi_l,
        Catch::Matchers::WithinAbs(0.2,0.01));
    REQUIRE_THAT(wheel_vel.phi_r,
        Catch::Matchers::WithinAbs(0.2,0.01));

    // Test pure rotation
    Twist2D twist1{PI/2, 0.0, 0.0};
    Wheel_state wheel_vel1 = dd.inverse_kinematics(twist1);
    REQUIRE_THAT(wheel_vel1.phi_l,
        Catch::Matchers::WithinAbs(-PI/4,0.01));
    REQUIRE_THAT(wheel_vel1.phi_r,
        Catch::Matchers::WithinAbs(PI/4,0.01));

    // Test : arc of a circle
    Twist2D twist2{PI/2, 0.3, 0.0};
    Wheel_state wheel_vel2 = dd.inverse_kinematics(twist2);
    REQUIRE_THAT(wheel_vel2.phi_l,
        Catch::Matchers::WithinAbs(-0.5,0.1));
    REQUIRE_THAT(wheel_vel2.phi_r,
        Catch::Matchers::WithinAbs(1.0,0.1));

    Twist2D twist3{0.0, 0.0, 3.0};

    //Syntax done with copilot
    REQUIRE_THROWS_AS(dd.inverse_kinematics(twist3), std::logic_error);


}