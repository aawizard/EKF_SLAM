#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include"turtlelib/geometry2d.hpp"
#include"turtlelib/se2d.hpp"
#include<sstream>
#include<cmath>
// #include <catch2/catch.hpp>


using namespace turtlelib;

TEST_CASE("Test operator<<", "[operator<<TWIST]"){
    Twist2D tw;
    tw.omega = 1.0;
    tw.x = 2.2;
    tw.y = 3.3;
    std::stringstream ss;
    ss << tw;
    REQUIRE(ss.str() == "[1 2.2 3.3]");
}

TEST_CASE("Test operator>>", "[operator>> TWIAT]"){
    Twist2D t;
    std::stringstream ss("[2.0 1 2.2]");
    ss >> t;
    REQUIRE(t.omega == 2.0);
    REQUIRE(t.x == 1.0);
    REQUIRE(t.y == 2.2);

    std::stringstream ss1("2.0 1 2.0");
    ss1 >> t;
    REQUIRE(t.omega == 2.0);
    REQUIRE(t.x == 1.0);
    REQUIRE(t.y == 2.0);
}



TEST_CASE("Transform2D Default Constructor", "[Transform2D]") {
    Transform2D tf;
    Vector2D v = tf.translation();
    REQUIRE(v.x == 0.0);
    REQUIRE(v.y == 0.0);
    REQUIRE(tf.rotation() == 0.0);
}

TEST_CASE("Transform2D Pure Translation Constructor", "[Transform2D_TRANS]") {
    Vector2D translation;
    translation.x = 1.0;
    translation.y = 2.0;
    Transform2D tf(translation);
    Vector2D v = tf.translation();
    REQUIRE(v.x == 1.0);
    REQUIRE(v.y == 2.0);
    REQUIRE(tf.rotation() == 0.0);
}

TEST_CASE("Transform2D Pure Rotation Constructor", "[Transform2D_ROT]") {
    double radians = 2.0;
    Transform2D tf(radians);
    Vector2D v = tf.translation();
    REQUIRE(v.x == 0.0);
    REQUIRE(v.y == 0.0);
    REQUIRE(tf.rotation() == radians);
}

TEST_CASE("Transform2D Pure Translation Constructor", "[Transform2D_TRANS_ROT]") {
    Vector2D translation;
    translation.x = 1.0;
    translation.y = 2.0;
    Transform2D tf(translation, 3.0);
    Vector2D v = tf.translation();
    REQUIRE(v.x == 1.0);
    REQUIRE(v.y == 2.0);
    REQUIRE(tf.rotation() == 3.0);
}

TEST_CASE("Transform2D operator()", "[Transform2D_operator() point]") {
    Vector2D translation;
    translation.x = 1.0;
    translation.y = 2.0;
    Point2D p;
    p.x = 1.0;
    p.y = 2.0;
    Transform2D tf(translation);
    Point2D p1 = tf(p);
    REQUIRE(p1.x == 2.0);
    REQUIRE(p1.y == 4.0);



    Transform2D tf1(translation, PI);
    Point2D p2 = tf1(p);
    REQUIRE(almost_equal(p2.x, 0.0));
    REQUIRE(almost_equal(p2.y, 0.0));
}

TEST_CASE("Transform2D operator()", "[Transform2D_operator() vector]") {
    Vector2D translation;
    translation.x = 1.0;
    translation.y = 2.0;
    Vector2D v;
    v.x = 1.0;
    v.y = 2.0;
    Transform2D tf(translation);
    Vector2D v1 = tf(v);
    REQUIRE(v1.x == 1.0);
    REQUIRE(v1.y == 2.0);

    Transform2D tf1(translation, PI);
    Vector2D v2 = tf1(v);
    REQUIRE(almost_equal(v2.x, -1.0));
    REQUIRE(almost_equal(v2.y, -2.0));
}

TEST_CASE("testing () operator for twist", "[twist()]"){ //Shail Dalal
        Twist2D twi{PI/2, 1.4, 1.1};
        Transform2D trans{Vector2D{2.2,3.3}, PI/2};
        twi = trans.operator()(twi);
        REQUIRE_THAT(twi.omega,
        Catch::Matchers::WithinAbs(PI/2, 0.1));
        REQUIRE_THAT(twi.x, 
        Catch::Matchers::WithinAbs(4.081, 0.1));
        REQUIRE_THAT(twi.y, 
        Catch::Matchers::WithinAbs(-2.057, 0.1));
    }


TEST_CASE("Transform2D inv()", "[Transform2D_inv]") {
    Vector2D translation = {3.0, 2.0};
    Transform2D tf(translation);
    Transform2D tf1 = tf.inv();
    Vector2D v = tf1.translation();
    REQUIRE(v.x == -3.0);
    REQUIRE(v.y == -2.0);
    REQUIRE(tf1.rotation() == 0.0);
    
    Transform2D tf2(translation, PI/2);
    Transform2D tf3 = tf2.inv();
    Vector2D v1 = tf3.translation();
    REQUIRE(v1.x == -2.0);
    REQUIRE(v1.y == 3.0);
    REQUIRE(tf3.rotation() == -PI/2);
}



TEST_CASE("testing * operator for two transform2D objects", "[multiplication*]"){ //Shail Dalal
    Transform2D lhs{Vector2D{3.0,5.2}, PI/2};
    Transform2D rhs{Vector2D{3.7,6.3}, PI/4};
    lhs = operator*(lhs,rhs);

    REQUIRE(lhs.rotation() == 3*(PI/4));
    REQUIRE_THAT(lhs.translation().x,
        Catch::Matchers::WithinAbs(-3.3,0.1));
    REQUIRE_THAT(lhs.translation().y,
        Catch::Matchers::WithinAbs(8.9,0.1));
    }


TEST_CASE("Test operator<<", "[operator<<Transform2D]"){
    Vector2D p{1.0, 2.2};
    Transform2D tf(p, PI/2);
    std::stringstream ss;
    ss << tf;
    REQUIRE(ss.str() == "deg: 90 x: 1 y: 2.2");
}

TEST_CASE("Test operator>>", "[operator>> Transform2D]"){
    Transform2D t;
    
    std::stringstream ss("deg: 180 x: 1 y: 2.2");
    ss >> t;
    REQUIRE_THAT(t.rotation(),
        Catch::Matchers::WithinAbs(PI,0.1));
    REQUIRE(t.translation().x == 1.0);
    REQUIRE(t.translation().y == 2.2);

    std::stringstream ss1("180 1.0 2.2");
    ss1 >> t;
    REQUIRE_THAT(t.rotation(),
        Catch::Matchers::WithinAbs(PI,0.1));
    REQUIRE(t.translation().x == 1.0);
    REQUIRE(t.translation().y == 2.2);
}

TEST_CASE("Test integrate_twist", "[integrate_twist]"){
    
    //Pure Rotation
    Twist2D tw;
    tw.omega = PI/2;
    Transform2D tf = integrate_twist(tw);
    REQUIRE(tf.rotation() == PI/2);
    REQUIRE(tf.translation().x == 0.0);
    REQUIRE(tf.translation().y == 0.0);

    //Pure Translation
    Twist2D tw1 = Twist2D{0.0, 1.0, 2.0};
    Transform2D tf2 = integrate_twist(tw1);
    REQUIRE_THAT(tf2.rotation(),
        Catch::Matchers::WithinAbs(0.0,0.1));
    REQUIRE_THAT(tf2.translation().x,
        Catch::Matchers::WithinAbs(1.0,0.1));
    REQUIRE_THAT(tf2.translation().y,
        Catch::Matchers::WithinAbs(2.0,0.1));


    //Rotation and Translation
    Twist2D tw3 = Twist2D{-1.24, -2.15,-2.92};
    Transform2D tf3 = integrate_twist(tw3);
    REQUIRE_THAT(tf3.rotation(),
        Catch::Matchers::WithinAbs(-1.24,0.1));
    REQUIRE_THAT(tf3.translation().x,
        Catch::Matchers::WithinAbs(-3.229863264722,0.2));
    REQUIRE_THAT(tf3.translation().y,
        Catch::Matchers::WithinAbs(-1.05645265317421,0.2));
}