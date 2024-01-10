#include"turtlelib/geometry2d.hpp"

namespace turtlelib {
    double normalize_angle(double rad){
        if (rad > PI){
            rad -= int(rad / (2 * PI))*2*PI;
        }
        else if (rad <= -PI){
            rad += int(rad / (2 * PI))*2*PI;

        }
        return rad;
    }
};

