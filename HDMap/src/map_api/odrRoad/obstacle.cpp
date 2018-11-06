/********************************************************
*   Copyright (C) 2018 All rights reserved.
*   
*   Filename:obstacle.cpp
*   Author  :lubing.han
*   Date    :2018-01-16
*   Describe:
*
********************************************************/
#include "obstacle.h"
using namespace std;

namespace opendrive
{

vector<object::shape::Point> Obstacle::getGroundPoints() const {
	if (getShape().size() != 1) return vector<object::shape::Point>();
	auto& shape = getShape()[0];
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

}
