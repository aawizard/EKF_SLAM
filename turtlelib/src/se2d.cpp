
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
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
    Transform2D::Transform2D(){
        point.x=0;
        point.y=0;
        theta=0;
    }
    Transform2D::Transform2D(Vector2D trans){
        point.x=trans.x;
        point.y=trans.y;
        theta=0;
    }
    Transform2D::Transform2D(double radians){
        point.x=0;
        point.y=0;
        theta=radians;
    }

    Transform2D::Transform2D(Vector2D trans, double radians){
        point.x = trans.x;
        point.y= trans.y;
        theta=radians;
    }

    Point2D Transform2D::operator()(Point2D p) const
    {
        Point2D p1;
        p1.x = p.x * cos(theta) - p.y * sin(theta) + point.x;
        p1.y = p.x * sin(theta) + p.y * cos(theta) + point.y;
        return p1;
    }
    Vector2D Transform2D::operator()(Vector2D v) const{
        return {(v.x * cos(theta)) - (v.y * sin(theta)), (v.x * sin(theta)) + (v.y * cos(theta))};
    }
    Twist2D Transform2D::operator()(Twist2D v) const{
        Twist2D t;
        t.omega = v.omega;
        t.x = (v.x * cos(theta)) - (v.y * sin(theta)) + (point.y*v.omega);
        t.y = (v.x * sin(theta)) + (v.y * cos(theta)) - (point.x*v.omega);
        return t;
    }

    Transform2D Transform2D::inv() const{
        Transform2D t;
        t.point.x = -point.x * cos(theta) - point.y * sin(theta);
        t.point.y = point.x * sin(theta) - point.y * cos(theta);
        t.theta = -theta;
        return t;
    }
    Transform2D & Transform2D::operator*=(const Transform2D & rhs){
        Transform2D t;
        t.point.x = rhs.point.x * cos(theta) - rhs.point.y * sin(theta) + point.x;
        t.point.y = rhs.point.x * sin(theta) + rhs.point.y * cos(theta) + point.y;
        t.theta = theta + rhs.theta;
        
        point.x = t.point.x;
        point.y = t.point.y;
        theta = t.theta;
        return *this;
    }

    Vector2D Transform2D::translation() const{
        return {point.x,point.y};
    }
    double Transform2D::rotation() const{
        return theta;
    }

    std::ostream & operator<<(std::ostream & os, const Transform2D & tf){
        std::stringstream ss;
        
        ss << "deg: " << rad2deg(tf.rotation()) << " x: " << tf.point.x << " y: " << tf.point.y;
        return os << ss.str();
    }
    std::istream & operator>>(std::istream & is, Transform2D & tf){
        const auto c1 = is.get();     
        double deg ;
        Vector2D v ;

        if (c1 == 'd'){
            std::string c3 ,c4, c5;
            is >> c3 >> deg >> c4 >> v.x >> c5 >> v.y; 
        }
        else{
            is.putback(c1);
            is >> deg >> v.x >> v.y;
        }
        Transform2D new_o{v,deg2rad(deg)};
        tf = new_o;
        return is;
    }
    Transform2D operator*(Transform2D lhs, const Transform2D & rhs){
        lhs *= rhs;
        return lhs;
    }

    Transform2D integrate_twist(Twist2D v){
        //Trandform of the body in one unit time (s)
        if (abs(v.omega) < 0.0001){
            Transform2D t({v.x,v.y},v.omega);

            return t;
            }
        else{
            Transform2D Tsb({v.y/v.omega,-v.x/v.omega},0.0);
            Transform2D Tss_({0,0},v.omega);
            Transform2D Tbs = Tsb.inv();
            Transform2D Tbb_ = Tbs * Tss_ * Tsb;
            return Tbb_;
        }
    }
};

