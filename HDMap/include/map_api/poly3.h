/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: poly3.h
*   Author  : lubing.han
*   Date    : 2016-11-15
*   Describe:
*
********************************************************/
#ifndef POLY3_H
#define POLY3_H
#include <vector>
#include "geometry.h"
using namespace std;

namespace opendrive
{

class Poly3: public Geometry
{
public:
	Poly3() {};
	Poly3(double a, double b, double c, double d): _a(a), _b(b), _c(c), _d(d) {};
	Poly3(Point& start, double len, double hdg, double a, double b, double c, double d);
	~Poly3() {};
	virtual void get_points(vector<Point> &points, double step);
	virtual Geometry* clone();
private:
	double _a;
	double _b;
	double _c;
	double _d;
};

}

#endif