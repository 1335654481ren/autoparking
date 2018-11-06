/********************************************************
*   Copyright (C) 2018 All rights reserved.
*
*   Filename: opendrive_object_shape_column.h
*   Author  : lubing.han
*   Date    : 2018-5-16
*   Describe: 
*
********************************************************/
#ifndef OPENDRIVE_OBJECT_SHAPE_COLUMN_H
#define OPENDRIVE_OBJECT_SHAPE_COLUMN_H

#include "tinyxml2.h"
#include "opendrive_object_shape_curve.h"
#include "opendrive_object_shape_polygon.h"
#include "opendrive_object_shape_column_verticaledge.h"
#ifndef DEROS
#include <autodrive_msgs/opendrive_object_shape_column_msg.h>
#else
#include "autodrive_msgs_opendrive_object_shape_column_msg.h"
#endif
#include <vector>
#include <iostream>
using namespace std;

namespace opendrive {
namespace object {
namespace shape {

class Column
{
public:
	Column();
	~Column();

	double& getHeight() {return _height;}
	const double& getHeight() const {return _height;}
	void setHeight(const double &height) {_height = height;}

	bool hasCurve() const {return _hasCurve;}
	void setHasCurve(bool hasCurve) {_hasCurve = hasCurve;}
	Curve& getCurve() {return _curve;}
	const Curve& getCurve() const {return _curve;}
	void setCurve(const Curve &curve) {_curve = curve;}

	bool hasPolygon() const {return _hasPolygon;}
	void setHasPolygon(bool hasPolygon) {_hasPolygon = hasPolygon;}
	Polygon& getPolygon() {return _polygon;}
	const Polygon& getPolygon() const {return _polygon;}
	void setPolygon(const Polygon &polygon) {_polygon = polygon;}

	bool hasVerticalEdge() const {return _hasVerticalEdge;}
	void setHasVerticalEdge(bool hasVerticalEdge) {_hasVerticalEdge = hasVerticalEdge;}
	column::VerticalEdge& getVerticalEdge() {return _verticalEdge;}
	const column::VerticalEdge& getVerticalEdge() const {return _verticalEdge;}
	void setVerticalEdge(const column::VerticalEdge &verticalEdge) {_verticalEdge = verticalEdge;}

	static Column fromXML(const tinyxml2::XMLElement* ele);
	tinyxml2::XMLElement* toXML(tinyxml2::XMLDocument* xmlDoc, string tagName) const;
	static Column fromMsg(const autodrive_msgs::opendrive_object_shape_column_msg& msg);
	autodrive_msgs::opendrive_object_shape_column_msg toMsg() const;
	void print(string prefix = "", int level = 10) const;
	bool check() const;
private:
	double _height;
	bool _hasCurve;
	Curve _curve;
	bool _hasPolygon;
	Polygon _polygon;
	bool _hasVerticalEdge;
	column::VerticalEdge _verticalEdge;
};
} // namespace shape
} // namespace object
} // namespace opendrive

#endif
