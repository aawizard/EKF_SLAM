#include <catch2/catch_test_macros.hpp>
#include"turtlelib/geometry2d.hpp"
using namespace turtlelib;

TEST_CASE("Test  Normalized angles", "[normalize_angle]"){
    // REQUIRE(normalize_angle(0.0) == 0.0);
    // REQUIRE(normalize_angle(PI) == PI);
    // REQUIRE(normalize_angle(4*PI) == 0.0);
    REQUIRE(normalize_angle(2.0) == 1.0);
}