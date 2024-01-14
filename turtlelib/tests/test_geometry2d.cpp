#include <catch2/catch_test_macros.hpp>
#include"turtlelib/geometry2d.hpp"
#include<sstream>
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

TEST_CASE("Test operator<<", "[operator<< point]"){
    Point2D p;
    p.x = 1.0;
    p.y = 2.2;
    std::stringstream ss;
    ss << p;
    REQUIRE(ss.str() == "[1 2.2]");
}

TEST_CASE("Test operator>>", "[operator>> point]"){
    Point2D p;
    std::stringstream ss("[1 2.2]");
    ss >> p;
    REQUIRE(p.x == 1.0);
    REQUIRE(p.y == 2.2);

    std::stringstream ss1("1 2.0");
    ss1 >> p;
    REQUIRE(p.x == 1.0);
    REQUIRE(p.y == 2.0);
}

TEST_CASE("Test operator +","[operator+]"){
    Point2D p;
    p.x = 1.0;
    p.y = 2.0;
    Vector2D v;
    v.x = 3.0;
    v.y = 4.0;
    Point2D p1 = p + v;
    REQUIRE(p1.x == 4.0);
    REQUIRE(p1.y == 6.0);
}

TEST_CASE("Test operator -","[operator-]"){
    Point2D p;
    p.x = 1.0;
    p.y = 2.0;
    Point2D p1;
    p1.x = 3.0;
    p1.y = 4.0;
    Vector2D v = p1 - p;
    REQUIRE(v.x == 2.0);
    REQUIRE(v.y == 2.0);
}

TEST_CASE("Test operator<<", "[operator<< vector]"){
    Vector2D v;
    v.x = 1.0;
    v.y = 2.2;
    std::stringstream ss;
    ss << v;
    REQUIRE(ss.str() == "[1 2.2]");
}

TEST_CASE("Test operator>>", "[operator>> vector]"){
    Vector2D v;
    std::stringstream ss("[1 2.2]");
    ss >> v;
    REQUIRE(v.x == 1.0);
    REQUIRE(v.y == 2.2);

    std::stringstream ss1("1 2.0");
    ss1 >> v;
    REQUIRE(v.x == 1.0);
    REQUIRE(v.y == 2.0);
}