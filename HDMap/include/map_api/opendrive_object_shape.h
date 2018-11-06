/********************************************************
*   Copyright (C) 2018 All rights reserved.
*
*   Filename: opendrive_object_shape.h
*   Author  : lubing.han
*   Date    : 2018-5-16
*   Describe: 
*
********************************************************/
#ifndef OPENDRIVE_OBJECT_SHAPE_H
#define OPENDRIVE_OBJECT_SHAPE_H

#include "tinyxml2.h"
#include "opendrive_object_shape_curve.h"
#include "opendrive_object_shape_polygon.h"
#include "opendrive_object_shape_column.h"
#ifndef DEROS
#include <autodrive_msgs/opendrive_object_shape_msg.h>
#else
#include "autodrive_msgs_opendrive_object_shape_msg.h"
#endif
#include <vector>
#include <iostream>
using namespace std;

namespace opendrive {
namespace object {

class Shape
{
public:
	Shape();
	~Shape();

	bool hasCurve() const {return _hasCurve;}
	void setHasCurve(bool hasCurve) {_hasCurve = hasCurve;}
	shape::Curve& getCurve() {return _curve;}
	const shape::Curve& getCurve() const {return _curve;}
	void setCurve(const shape::Curve &curve) {_curve = curve;}

	bool hasPolygon() const {return _hasPolygon;}
	void setHasPolygon(bool hasPolygon) {_hasPolygon = hasPolygon;}
	shape::Polygon& getPolygon() {return _polygon;}
	const shape::Polygon& getPolygon() const {return _polygon;}
	void setPolygon(const shape::Polygon &polygon) {_polygon = polygon;}

	bool hasColumn() const {return _hasColumn;}
	void setHasColumn(bool hasColumn) {_hasColumn = hasColumn;}
	shape::Column& getColumn() {return _column;}
	const shape::Column& getColumn() const {return _column;}
	void setColumn(const shape::Column &column) {_column = column;}

	static Shape fromXML(const tinyxml2::XMLElement* ele);
	tinyxml2::XMLElement* toXML(tinyxml2::XMLDocument* xmlDoc, string tagName) const;
	static Shape fromMsg(const autodrive_msgs::opendrive_object_shape_msg& msg);
	autodrive_msgs::opendrive_object_shape_msg toMsg() const;
	void print(string prefix = "", int level = 10) const;
	bool check() const;
private:
	bool _hasCurve;
	shape::Curve _curve;
	bool _hasPolygon;
	shape::Polygon _polygon;
	bool _hasColumn;
	shape::Column _column;
};
} // namespace object
} // namespace opendrive

#endif
