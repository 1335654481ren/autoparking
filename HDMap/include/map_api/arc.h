/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: arc.h
*   Author  : lubing.han
*   Date    : 2016-11-15
*   Describe:
*
********************************************************/
#ifndef ARC_H
#define ARC_H

#include <vector>
#include "geometry.h"
using namespace std;

namespace opendrive
{

class Arc: public Geometry
{
public:
	Arc() {};
	Arc(double c): _curvature(c) {Geometry();};
	Arc(Point& start, double len, double hdg, double c);
	~Arc() {};
	virtual void get_points(vector<Point> &points, double step);
	virtual Geometry* clone();
private:
	double _curvature;
};

}

#endif