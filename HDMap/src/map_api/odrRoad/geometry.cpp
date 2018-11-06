/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename:geometry.cpp
*   Author  :weiwei.liu
*   Date    :2016-11-14
*   Describe:
*
********************************************************/

#include "geometry.h"
#include <math.h>

namespace opendrive{

double theta_unify(double theta) {
    if (theta > 2 * M_PI) {
        return theta - 2 * M_PI;
    }
    if (theta < 0) {
        return theta + 2 * M_PI;
    }
    return theta;
}

Point::Point() {
    x = y = z = s = l = theta = k = 0;
}

Point::Point(double _x, double _y, double _z) {
    x = _x;
    y = _y;
    z = _z;
    s = l = theta = k = 0;
}

Point::Point(const Point &point) {
    x = point.x;
    y = point.y;
    z = point.z;
    s = point.s;
    l = point.l;
    theta = point.theta;
    k = point.k;
}

Point::Point(double _x, double _y, double _z, double _s, double _theta) {
    x = _x;
    y = _y;
    z = _z;
    s = _s;
    l = 0;
    theta = _theta;
    k = 0;
}

Point::Point(double _x, double _y, double _z, double _s, double _l, double _theta) {
    x = _x;
    y = _y;
    z = _z;
    s = _s;
    l = _l;
    theta = _theta;
    k = 0;
}

Point::Point(double _x, double _y, double _z, double _s, double _l, double _theta, double _k) {
    x = _x;
    y = _y;
    z = _z;
    s = _s;
    l = _l;
    theta = _theta;
    k = _k;
}

Point::~Point() {
}

Point& Point::operator=(const Point& point) {
    x = point.x;
    y = point.y;
    z = point.z;
    s = point.s;
    l = point.l;
    theta = point.theta;
    k = point.k;
    return *this;
}

void Point::print() const {
    std::cout << "(" << x << ", " << y << ", " << z << "), (" << s << ", " << l << ", " << theta << ", " << k << ")" << std::endl;
}

double Point::_distance(const Point& p) const {
    double dx = x - p.x;
    double dy = y - p.y;
    double dz = z - p.z;
    return sqrt(dx * dx + dy * dy + dz * dz);
}

double Point::dist2(const Point& p) const {
    return (x - p.x) * (x - p.x) + (y - p.y) * (y - p.y) + (z - p.z) * (z - p.z);
}

double Point::dist2xy(const Point& p) const {
    return (x - p.x) * (x - p.x) + (y - p.y) * (y - p.y);
}

double Point::dist2(const Point& p1, const Point& p2, double& t) const {
    Point AB = p2 - p1;
    Point AC = *this - p1;
    double dAB2 = p2.dist2(p1);
    t = AB*AC / dAB2;
    t = t > 1 ? 1 : t;
    t = t < 0 ? 0 : t;
    Point r = (1-t)*p1 + t*p2;
    return dist2(r);
}

double Point::dist2xy(const Point& p1, const Point& p2, double& t) const {
    Point pp1 = p1, pp2 = p2;
    pp1.z = pp2.z = z;
    return this->dist2(pp1, pp2, t);
}

double Point::yaw() const {
    if (x > 0)
        return atan(y/x);
    else if (x < 0)
        return M_PI - atan(-y/x);
    else
        return y < 0 ? -M_PI/2 : M_PI/2;
}

Point operator+(const Point &p1, const Point &p2) {
    Point r;
    r.x = p1.x + p2.x;
    r.y = p1.y + p2.y;
    r.z = p1.z + p2.z;
    r.s = p1.s + p2.s;
    r.l = p1.l + p2.l;
    r.theta = p1.theta + p2.theta;
    r.k = p1.k + p2.k;
    return r;
}

Point operator-(const Point &p1, const Point &p2) {
    Point r;
    r.x = p1.x - p2.x;
    r.y = p1.y - p2.y;
    r.z = p1.z - p2.z;
    r.s = p1.s - p2.s;
    r.l = p1.l - p2.l;
    r.theta = p1.theta - p2.theta;
    r.k = p1.k - p2.k;
    return r;
}

Point operator*(const double &d, const Point &p) {
    Point r;
    r.x = d * p.x;
    r.y = d * p.y;
    r.z = d * p.z;
    r.s = d * p.s;
    r.l = d * p.l;
    r.theta = d * p.theta;
    r.k = d * p.k;
    return r;
}

Point operator*(const Point &p, const double &d) {
    Point r;
    r.x = d * p.x;
    r.y = d * p.y;
    r.z = d * p.z;
    r.s = d * p.s;
    r.l = d * p.l;
    r.theta = d * p.theta;
    r.k = d * p.k;
    return r;
}

double operator*(const Point &p1, const Point &p2) {
    return p1.x*p2.x + p1.y*p2.y + p1.z*p2.z;
}

Geometry::Geometry(){
    _hdg = 0.0;
    _len = 0.0;
}

Geometry::~Geometry() {
}

Geometry::Geometry(Point start, double len, double hdg) {
    _start = start;
    _len = len;
    _hdg = hdg;
}

double Geometry::get_hdg(){
    return _hdg;
}

double Geometry::get_len(){
    return _len;
}

Point Geometry::get_start(){
    return _start;
}

void Geometry::set_hdg(double& hdg){
    _hdg = hdg;
}

void Geometry::set_len(double& len){
    _len = len;
}

void Geometry::set_start(Point& start){
    _start = start;
}

void Geometry::get_points(std::vector<Point>& points, double density){
    // nothing to do here
}

Geometry* Geometry::clone() {
    Geometry* geometry = new Geometry();
    *geometry = *this;
    return geometry;
}

void Geometry::get_curvatures(std::vector<Point>& points) {
    if (points.size() < 3) return;
    for (int i = 1; i < points.size() - 1; ++i) {
        Point p12 = points[i] - points[i - 1];
        Point p23 = points[i + 1] - points[i];
        double aved = (sqrt(points[i].dist2(points[i - 1])) + sqrt(points[i + 1].dist2(points[i]))) / 2;
        double dt = p23.yaw() - p12.yaw();
        if (dt > M_PI) dt -= 2*M_PI;
        if (dt < -M_PI) dt += 2*M_PI;
        points[i].k = dt / aved;
    }
    points[0].k = points[1].k;
    points[points.size() - 1].k = points[points.size() - 2].k;
}

} //end namespace opendrive


