/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: planview.h
*   Author  : lubing.han
*   Date    : 2016-11-15
*   Describe:
*
********************************************************/
#ifndef PLANVIEW_H
#define PLANVIEW_H

#include "geometry.h"
#include <vector>
using namespace std;

namespace opendrive
{

class PlanView
{
public:
	PlanView();
	~PlanView();
	PlanView(const PlanView& planview);
	PlanView& operator=(const PlanView& planview);
	void append_geometry(Geometry* pgeo);
	vector<Point> get_refline(double step);
private:
	vector<Geometry*> geometries;
};

}

#endif