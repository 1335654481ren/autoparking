/********************************************************
*   Copyright (C) 2018 All rights reserved.
*   
*   Filename: object_fun.cpp
*   Author  : lubing.han
*   Date    : 2018-02-27
*   Describe: object_fun.cpp
*
********************************************************/
#include "object_fun.h"
#include <math.h>
using namespace std;

namespace opendrive {

vector<object::shape::Point> getGroundPoints(const Object& object) {
	auto& shape = object.getShape()[0];
	if (shape.hasColumn() && shape.getColumn().hasPolygon())
		return shape.getColumn().getPolygon().getPoint();
	else if (shape.hasColumn() && shape.getColumn().hasCurve())
		return shape.getColumn().getCurve().getPoint();
	else if (shape.hasPolygon())
		return shape.getPolygon().getPoint();
	else if (shape.hasCurve())
		return shape.getCurve().getPoint();
	else
		return vector<object::shape::Point>();
}

bool isClosed(const Object& o) {
	return o.getShape()[0].hasPolygon() || o.getShape()[0].hasColumn() && o.getShape()[0].getColumn().hasPolygon();
}

SegmentList toSegmentList(const Object& object) {
	SegmentList sl;
	sl.setId(atoi(object.getId().c_str()));
	sl.setType(object.getType());
	sl.setSubtype(object.getSubtype());
	if (object.getShape()[0].hasColumn())
		sl.setHeight(object.getShape()[0].getColumn().getHeight());
	else sl.setHeight(0.0);
	vector<object::shape::Point> points = getGroundPoints(object);
	sl.appendSegments(points, isClosed(object));
	return sl;
}

}