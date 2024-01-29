#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include"turtlelib/geometry2d.hpp"
#include<sstream>
#include<cmath>
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

TEST_CASE("Test normalize_vector", "[normalize_vector]"){
    Vector2D v;
    v.x = 1.0;
    v.y = 1.0;
    Vector2D v1 = normalize_vector(v);
    REQUIRE_THAT(v1.x,
        Catch::Matchers::WithinAbs(0.8, 0.1));
    REQUIRE_THAT(v1.y,
        Catch::Matchers::WithinAbs(0.8, 0.1));
}

TEST_CASE("Test operator -=","[operator-= vector]"){
    Vector2D v{1.0,2.0};
    Vector2D v1{3.0,4.0};
    v1 -= v;
    REQUIRE(v1.x == 2.0);
    REQUIRE(v1.y == 2.0);
}

TEST_CASE("Test operator +=","[operator+= vector]"){
    Vector2D v{1.0,2.0};
    Vector2D v1{3.0,4.0};
    v1 += v;
    REQUIRE(v1.x == 4.0);
    REQUIRE(v1.y == 6.0);
}

TEST_CASE("Test operator *=","[operator*= vector]"){
    Vector2D v{1.0,2.0};
    v *= 2;
    REQUIRE(v.x == 2.0);
    REQUIRE(v.y == 4.0);
}

TEST_CASE("Test operator -","[operator- vector]"){
    Vector2D v{1.0,2.0};
    Vector2D v1{3.0,4.0};
    Vector2D v2 = v1 - v;
    REQUIRE(v2.x == 2.0);
    REQUIRE(v2.y == 2.0);
}

TEST_CASE("Test operator +","[operator+ vector]"){
    Vector2D v{1.0,2.0};
    Vector2D v1{3.0,4.0};
    Vector2D v2 = v1 + v;
    REQUIRE(v2.x == 4.0);
    REQUIRE(v2.y == 6.0);
}

TEST_CASE("Test operator *","[operator* vector]"){
    Vector2D v{1.0,2.0};
    Vector2D v1 = v * 2;
    REQUIRE(v1.x == 2.0);
    REQUIRE(v1.y == 4.0);
}

TEST_CASE("Test operator *","[operator* vector1]"){
    Vector2D v{1.0,2.0};
    Vector2D v1 = 2 * v;
    REQUIRE(v1.x == 2.0);
    REQUIRE(v1.y == 4.0);
}


TEST_CASE("Test dot","[dot vector]"){
    Vector2D v{1.0,2.0};
    Vector2D v1{3.0,4.0};
    double d = dot(v,v1);
    REQUIRE(d == 11.0);
}

TEST_CASE("Test magnitude","[magnitude vector]"){
    Vector2D v{1.0,2.0};
    double d = magnitude(v);
    REQUIRE(d == sqrt(5.0));
}
TEST_CASE("Test angle","[angle vector]"){
    Vector2D v{1.0,2.0};
    Vector2D v1{3.0,4.0};
    double d = angle(v,v1);
    REQUIRE(d == 0.17985349979247847);
}

