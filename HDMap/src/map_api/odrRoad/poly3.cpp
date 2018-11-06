/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: poly3.cpp
*   Author  : lubing.han
*   Date    : 2016-11-15
*   Describe:
*
********************************************************/
#include "poly3.h"
#include <math.h>
using namespace std;

namespace opendrive
{

Poly3::Poly3(Point& start, double len, double hdg, double a, double b, double c, double d): Geometry(start, len, hdg) {
	_a = a;
	_b = b;
	_c = c;
	_d = d;
}
	
void Poly3::get_points(vector<Point> &points, double step) {
	points.clear();
	double ov = _b;
	int n = int(get_len() / step) + 1;
	double x0 = get_start().x;
	double y0 = get_start().y;
	double z0 = get_start().z;
	double u = 0;
	for (int i = 0; i < n; ++ i) {
		double du = step / sqrt(1 + ov*ov);
		ov = _b + 2*_c*u + 3*_d*u*u;
		double v = _a + _b*u + _c*u*u + _d*u*u*u;
		double x = x0 + u * cos(get_hdg()) - v * sin(get_hdg());
		double y = y0 + u * sin(get_hdg()) + v * cos(get_hdg());

        // here: arctan(_b) + get_hdg() calculates theta of current point !
		Point p(x, y, z0, i * step, theta_unify( get_hdg() + atan(ov) ) );
		points.push_back(p);
		u += du;
	}
}

Geometry* Poly3::clone() {
	Poly3* poly3 = new Poly3();
	*poly3 = *this;
	return poly3;
}

}
