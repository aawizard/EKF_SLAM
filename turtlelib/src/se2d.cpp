#include"turtlelib/se2d.hpp"
#include<cmath>
#include<sstream>

namespace turtlelib {
    
    std::ostream & operator<<(std::ostream & os, const Twist2D & tw){
        std::stringstream ss;
        ss << "[" << tw.omega << " " << tw.x << " " << tw.y << "]";
        return os << ss.str();
    }

    std::istream & operator>>(std::istream & is, Twist2D & tw){
        char c;
    is >> c;
    if (c == '[')
    {
        is >>tw.omega>> tw.x >> tw.y >> c;
    }
    else
    {
        is.putback(c);
        is >>tw.omega>> tw.x >> tw.y;
    }
    return is;
    }

};

