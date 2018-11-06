/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: paramPoly3.h
*   Author  : weiwei.liu
*   Date    : 2016-11-15
*   Describe:
*
********************************************************/
#ifndef PARAMPOLY3_H
#define PARAMPOLY3_H
#include <vector>
#include "geometry.h"
using namespace std;

namespace opendrive
{

class ParamPoly3: public Geometry
{
public:
	ParamPoly3() {};
	ParamPoly3(Point& start, double len, double hdg, double au, double bu, double cu, double du
                                    ,double av, double bv, double cv, double dv);
	~ParamPoly3() {};
	virtual void get_points(vector<Point> &points, double step);
	virtual Geometry* clone();

private:
    double Speed(double t);
	double _au;
	double _bu;
	double _cu;
	double _du;
	double _av;
	double _bv;
	double _cv;
	double _dv;
};

}

#endif
