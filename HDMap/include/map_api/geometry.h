/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename:geometry.h
*   Author  :weiwei.liu
*   Date    :2016-11-14
*   Describe:
*
********************************************************/
#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <iostream>
#include <vector>
#include <math.h>

namespace opendrive {

double theta_unify(double theta);

class Point{
public:
    Point();
    Point(double x, double y, double z);
    Point(double x, double y, double z, double s, double theta);
    Point(double x, double y, double z, double s, double l, double theta);
    Point(double x, double y, double z, double s, double l, double theta, double k);
    Point(const Point& point);
    Point& operator=(const Point& point);
    ~Point();
    void print() const;
    double _distance(const Point& p) const;
    double dist2(const Point& p) const;
    double dist2xy(const Point& p) const;
    double dist2(const Point& p1, const Point& p2, double& t) const;
    double dist2xy(const Point& p1, const Point& p2, double& t) const;
    double yaw() const;
    friend Point operator+(const Point &p1, const Point &p2);
    friend Point operator-(const Point &p1, const Point &p2);
    friend Point operator*(const double &d, const Point &p);
    friend Point operator*(const Point &p, const double &d);
    friend double operator*(const Point &p1, const Point &p2);

    double x;
    double y;
    double z;
    double s;
    double l;
    double theta;
    double k;
};

class Geometry {

public:
    Geometry();
    Geometry(Point start, double len, double hdg);
    ~Geometry();
    double get_hdg();
    double get_len();
    Point get_start();
    void set_hdg(double &hdg);
    void set_len(double &len);
    void set_start(Point &start);

    // get points from geometry params
    virtual void get_points(std::vector<Point>& points, double density);
    virtual Geometry* clone();
    static void get_curvatures(std::vector<Point>& points);

private:
    Point _start;
    double _hdg;
    double _len;
};



}

#endif
