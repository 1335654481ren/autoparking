/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: arc.cpp
*   Author  : lubing.han
*   Date    : 2016-11-15
*   Describe:
*
********************************************************/
#include "arc.h"
#include <math.h>
using namespace std;

namespace opendrive
{

Arc::Arc(Point& start, double len, double hdg, double c): Geometry(start, len, hdg) {
	_curvature = c;
}
	
void Arc::get_points(vector<Point> &points, double step) {
	points.clear();
	double x0 = get_start().x - sin(get_hdg()) / _curvature;
	double y0 = get_start().y + cos(get_hdg()) / _curvature;
	double z0 = get_start().z;
	int n = int(get_len() / step) + 1;
	double delta = step * _curvature;
	for (int i = 0; i < n; ++ i) {
		double heading = get_hdg() + i * delta;
		double x = x0 + sin(heading) / _curvature;
		double y = y0 - cos(heading) / _curvature;
		Point p(x, y, z0, i * step, theta_unify(heading) );
		points.push_back(p);
	}
}

Geometry* Arc::clone() {
	Arc* arc = new Arc();
	*arc = *this;
	return arc;
}

}
