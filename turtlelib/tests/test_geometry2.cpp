#include <catch2/catch_test_macros.hpp>
#include"turtlelib/geometry2d.hpp"
using namespace turtlelib;

TEST_CASE("Test  Normalized angles", "[normalize_angle]"){
    REQUIRE(normalize_angle(0.0) == 0.0);
    REQUIRE(normalize_angle(PI) == PI);
    REQUIRE(normalize_angle(PI/4) == PI/4);
    REQUIRE(normalize_angle(-PI/4) == -PI/4);
    REQUIRE(normalize_angle(PI*3/2) == -PI/2);
    REQUIRE(normalize_angle(-PI) == PI);
    REQUIRE(normalize_angle(-PI*5/2) == -PI/2);
}