#include"turtlelib/geometry2d.hpp"
#include<cmath>

namespace turtlelib {
    double normalize_angle(double rad) {
        rad = fmod(rad, 2 * PI);
    if (rad > PI) {
        rad -= 2 * PI;
    } else if (rad <= -PI) {
        rad += 2 * PI;
    }
    return rad;
}

//  std::ostream & operator<<(std::ostream & os, const Point2D & p){

//     return os;
//  }



};

