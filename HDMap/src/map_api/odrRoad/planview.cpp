/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: planview.cpp
*   Author  : lubing.han
*   Date    : 2016-11-15
*   Describe:
*
********************************************************/

#include "planview.h"
using namespace std;

namespace opendrive
{

PlanView::PlanView() {}

PlanView::~PlanView() {
	for (int i = 0; i < geometries.size(); ++i)
		delete geometries[i];
	geometries.clear();
}

PlanView::PlanView(const PlanView& planview) {
	for (int i = 0; i < planview.geometries.size(); ++i) {
		Geometry* geometry = planview.geometries[i]->clone();
		geometries.push_back(geometry);
	}
}

PlanView& PlanView::operator=(const PlanView& planview) {
	if (this == &planview) return *this;
	for (int i = 0; i < geometries.size(); ++i)
		delete geometries[i];
	geometries.clear();
	for (int i = 0; i < planview.geometries.size(); ++i) {
		Geometry* geometry = planview.geometries[i]->clone();
		geometries.push_back(geometry);
	}
	return *this;
}

void PlanView::append_geometry(Geometry* pgeo) {
	geometries.push_back(pgeo);
}

vector<Point> PlanView::get_refline(double step) {
	vector<Point> result, tmp;
	double sumlen = 0.0;
	for (int i = 0; i < geometries.size(); ++i) {
		tmp.clear();
		geometries[i]->get_points(tmp, step);
		Geometry::get_curvatures(tmp);
		for (int j = 0; j < tmp.size(); ++j)
			tmp[j].s += sumlen;
		sumlen += geometries[i]->get_len();
		result.insert(result.end(), tmp.begin(), tmp.end());
	}
	return result;
}

}
