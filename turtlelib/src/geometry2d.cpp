
#include"turtlelib/geometry2d.hpp"
#include<cmath>
#include<sstream>

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

 std::ostream & operator<<(std::ostream & os, const Point2D & p){
    std::stringstream ss;
    ss << "[" << p.x << " " << p.y << "]";
    return os << ss.str();
 }
//Contributer: Demiana
 std::istream & operator>>(std::istream & is, Point2D & p){
     char c;
    is >> c;
    if (c == '[')
    {
        is >> p.x >> p.y >> c;
    }
    else
    {
        is.putback(c);
        is >> p.x >> p.y;
    }
    return is;
 }
Vector2D operator-(const Point2D & head, const Point2D & tail){
    Vector2D v;
    v.x = head.x - tail.x;
    v.y = head.y - tail.y;
    return v;
}
Point2D operator+(const Point2D & tail, const Vector2D & disp){
    Point2D p;
    p.x = tail.x + disp.x;
    p.y = tail.y + disp.y;
    return p;
    }

std::ostream & operator<<(std::ostream & os, const Vector2D & v){
    std::stringstream ss;
    ss << "[" << v.x << " " << v.y << "]";
    return os << ss.str();
    }

std::istream & operator>>(std::istream & is, Vector2D & v){
    char c;
    is >> c;
    if (c == '[')
    {
        is >> v.x >> v.y >> c;
    }
    else
    {
        is.putback(c);
        is >> v.x >> v.y;
    }
    return is;
}

Vector2D normalize_vector(const Vector2D & v)
    {
        Vector2D v1;
        v1.x = v.x / sqrt(v.x * v.x + v.y * v.y);
        v1.y = v.y / sqrt(v.x * v.x + v.y * v.y);
        return v1;
    }


Vector2D operator-(Vector2D & v1, const Vector2D & v2){
    v1 -= v2;
    return v1;
}


Vector2D & Vector2D::operator-=(const Vector2D & v){
    x -= v.x;
    y -= v.y;
    return *this;
}


Vector2D operator+(Vector2D & v1, const Vector2D & v2){
    v1 += v2;
    return v1;
}


Vector2D & Vector2D::operator+=(const Vector2D & v){
    x += v.x;
    y += v.y;
    return *this;
}


Vector2D operator*(Vector2D & v1, double d){
    v1 *= d;
    return v1;
}

Vector2D & Vector2D::operator*=(const double d){
    x *= d;
    y *= d;
    return *this;
}


double dot(Vector2D v1, Vector2D v2){
    return v1.x * v2.x + v1.y * v2.y;
}


double magnitude(Vector2D v){
    return sqrt(v.x * v.x + v.y * v.y);
}

double angle(Vector2D v1,Vector2D v2){
    return acos(dot(v1,v2)/(magnitude(v1)*magnitude(v2)));
}
};

