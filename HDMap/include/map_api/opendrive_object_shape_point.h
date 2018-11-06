/********************************************************
*   Copyright (C) 2018 All rights reserved.
*
*   Filename: opendrive_object_shape_point.h
*   Author  : lubing.han
*   Date    : 2018-5-16
*   Describe: 
*
********************************************************/
#ifndef OPENDRIVE_OBJECT_SHAPE_POINT_H
#define OPENDRIVE_OBJECT_SHAPE_POINT_H

#include "tinyxml2.h"
#include "opendrive_object_shape_point_segment.h"
#ifndef DEROS
#include <autodrive_msgs/opendrive_object_shape_point_msg.h>
#else
#include "autodrive_msgs_opendrive_object_shape_point_msg.h"
#endif
#include <vector>
#include <iostream>
using namespace std;

namespace opendrive {
namespace object {
namespace shape {

class Point
{
public:
	Point();
	~Point();

	double& getX() {return _x;}
	const double& getX() const {return _x;}
	void setX(const double &x) {_x = x;}

	double& getY() {return _y;}
	const double& getY() const {return _y;}
	void setY(const double &y) {_y = y;}

	double& getZ() {return _z;}
	const double& getZ() const {return _z;}
	void setZ(const double &z) {_z = z;}

	bool& getExact() {return _exact;}
	const bool& getExact() const {return _exact;}
	void setExact(const bool &exact) {_exact = exact;}

	bool hasSegment() const {return _hasSegment;}
	void setHasSegment(bool hasSegment) {_hasSegment = hasSegment;}
	point::Segment& getSegment() {return _segment;}
	const point::Segment& getSegment() const {return _segment;}
	void setSegment(const point::Segment &segment) {_segment = segment;}

	static Point fromXML(const tinyxml2::XMLElement* ele);
	tinyxml2::XMLElement* toXML(tinyxml2::XMLDocument* xmlDoc, string tagName) const;
	static Point fromMsg(const autodrive_msgs::opendrive_object_shape_point_msg& msg);
	autodrive_msgs::opendrive_object_shape_point_msg toMsg() const;
	void print(string prefix = "", int level = 10) const;
	bool check() const;
private:
	double _x;
	double _y;
	double _z;
	bool _exact;
	bool _hasSegment;
	point::Segment _segment;
};
} // namespace shape
} // namespace object
} // namespace opendrive

#endif
