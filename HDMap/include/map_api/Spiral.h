/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename:Spiral.h
*   Author  :weiwei.liu
*   Date    :2016-11-14
*   Describe:
*
********************************************************/
#ifndef SPIRAL_H
#define SPIRAL_H

#include "geometry.h"

namespace opendrive {

class Spiral : public Geometry {

public:
    Spiral();
    Spiral(Point& start, double len, double hdg, double start_cuv, double end_cuv);
    ~Spiral();
    void get_points(std::vector<Point>& points, double density);
    virtual Geometry* clone();
private:
    double _start_cuv;
    double _end_cuv;

};

}

#endif