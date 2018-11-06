/********************************************************
*   Copyright (C) 2018 All rights reserved.
*
*   Filename: opendrive_object_shape_curve.h
*   Author  : lubing.han
*   Date    : 2018-5-16
*   Describe: 
*
********************************************************/
#ifndef OPENDRIVE_OBJECT_SHAPE_CURVE_H
#define OPENDRIVE_OBJECT_SHAPE_CURVE_H

#include "tinyxml2.h"
#include "opendrive_object_shape_point.h"
#ifndef DEROS
#include <autodrive_msgs/opendrive_object_shape_curve_msg.h>
#else
#include "autodrive_msgs_opendrive_object_shape_curve_msg.h"
#endif
#include <vector>
#include <iostream>
using namespace std;

namespace opendrive {
namespace object {
namespace shape {

class Curve
{
public:
	Curve();
	~Curve();

	vector<Point>& getPoint() {return _point;}
	const vector<Point>& getPoint() const {return _point;}
	void setPoint(const vector<Point> &point) {_point = point;}
	void appendPoint(const Point &point) {_point.push_back(point);}

	static Curve fromXML(const tinyxml2::XMLElement* ele);
	tinyxml2::XMLElement* toXML(tinyxml2::XMLDocument* xmlDoc, string tagName) const;
	static Curve fromMsg(const autodrive_msgs::opendrive_object_shape_curve_msg& msg);
	autodrive_msgs::opendrive_object_shape_curve_msg toMsg() const;
	void print(string prefix = "", int level = 10) const;
	bool check() const;
private:
	vector<Point> _point;
};
} // namespace shape
} // namespace object
} // namespace opendrive

#endif
