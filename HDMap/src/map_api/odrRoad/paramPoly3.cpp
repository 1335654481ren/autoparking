/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: paramPoly3.cpp
*   Author  : lubing.han
*   Date    : 2016-11-15
*   Describe:
*
********************************************************/
#include "paramPoly3.h"
#include <math.h>
using namespace std;

namespace opendrive
{

ParamPoly3::ParamPoly3(Point& start, double len, double hdg, double au, double bu, double cu, double du
                           , double av, double bv, double cv, double dv):
            Geometry(start, len, hdg) {
	_au = au;
	_bu = bu;
	_cu = cu;
	_du = du;
    _av = av;
    _bv = bv;
    _cv = cv;
    _dv = dv;
}
	
void ParamPoly3::get_points(vector<Point> &points, double step) {
	points.clear();
	int n = int(get_len() / step) + 1;
	double x0 = get_start().x;
	double y0 = get_start().y;
	double z0 = get_start().z;

    double h = step / 10.0;
    double p = 0.0;
	for (int i = 0; p < 1.0; i++) {
        
        double k1 = h/Speed(p);
        double k2 = h/Speed(p + k1/2);
        double k3 = h/Speed(p + k2/2);
        double k4 = h/Speed(p + k3);
        p += (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0;
        if (i % 10 != 0 && p < 1.0) {
            continue;
        }
        
        double u = _au + _bu * p + _cu * p * p + _du * p * p * p;
        double v = _av + _bv * p + _cv * p * p + _dv * p * p * p;

        double ou = _bu + 2 * _cu * p + 3 * _du * p * p;
        double ov = _bv + 2 * _cv * p + 3 * _dv * p * p;

		double x = x0 + u * cos(get_hdg()) - v * sin(get_hdg());
		double y = y0 + u * sin(get_hdg()) + v * cos(get_hdg());

        // here: arctan(_b) + get_hdg() calculates theta of current point !
		double huv;
		if (ou == 0) huv = ov > 0 ? M_PI / 2 : - M_PI / 2;
		else huv = ou > 0 ? atan(ov/ou) : M_PI - atan(-ov/ou);
		Point P(x, y, z0, i * h, theta_unify( get_hdg() + huv ));
		points.push_back(P);
	}
}
    
    double ParamPoly3::Speed(double t) {
    double df_du = _bu + 2.0 * _cu * t + 3.0 * _du * t * t;
    double df_dv = _bv + 2.0 * _cv * t + 3.0 * _dv * t * t;
    return sqrt(df_du * df_du + df_dv * df_dv);
}

Geometry* ParamPoly3::clone() {
	ParamPoly3* paramPoly3 = new ParamPoly3();
	*paramPoly3 = *this;
	return paramPoly3;	
}

}
