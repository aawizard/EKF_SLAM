#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include"turtlelib/landmark_calc.hpp"
#include<armadillo>
#include<sstream>
#include<cmath>
// #include <catch2/catch.hpp>


using namespace turtlelib;

TEST_CASE("circle fit 1","[circle_fit]"){
    std::vector<std::vector<Vector2D>> clusters;
    std::vector<Vector2D> cluster;
    cluster.push_back(Vector2D{1.0, 7.0});
    cluster.push_back(Vector2D{2.0, 6.0});
    cluster.push_back(Vector2D{5.0, 8.0});
    cluster.push_back(Vector2D{7.0, 7.0});
    cluster.push_back(Vector2D{9.0, 5.0});
    cluster.push_back(Vector2D{3.0, 7.0});
    clusters.push_back(cluster);

    auto landmarklib = Landmarklib(clusters);
    auto landmark_centroids = landmarklib.get_landmark_centroids();
    REQUIRE_THAT(landmark_centroids[0][0], Catch::Matchers::WithinAbs(4.615482, 1e-4));
    REQUIRE_THAT(landmark_centroids[0][1], Catch::Matchers::WithinAbs(2.807354, 1e-4));
    REQUIRE_THAT(landmark_centroids[0][2], Catch::Matchers::WithinAbs(4.8275, 1e-4));
}

TEST_CASE("circle fit 2","[circle_fit]"){
    std::vector<std::vector<Vector2D>> clusters;
    std::vector<Vector2D> cluster;
    cluster.push_back(Vector2D{-1.0, 0.0});
    cluster.push_back(Vector2D{-0.3, -0.06});
    cluster.push_back(Vector2D{0.3, 0.1});
    cluster.push_back(Vector2D{1.0, 0.0});
    clusters.push_back(cluster);

    auto landmarklib = Landmarklib(clusters);
    auto landmark_centroids = landmarklib.get_landmark_centroids();
    REQUIRE_THAT(landmark_centroids[0][0], Catch::Matchers::WithinAbs(0.4908357, 1e-4));
    REQUIRE_THAT(landmark_centroids[0][1], Catch::Matchers::WithinAbs(-22.15212, 1e-4));
    REQUIRE_THAT(landmark_centroids[0][2], Catch::Matchers::WithinAbs(22.17979, 1e-4));
}