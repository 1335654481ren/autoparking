/********************************************************
*   Copyright (C) 2018 All rights reserved.
*
*   Filename: opendrive_object_shape.cpp
*   Author  : lubing.han
*   Date    : 2018-5-16
*   Describe: 
*
********************************************************/
#include "opendrive_object_shape.h"
using namespace std;

namespace opendrive {
namespace object {

Shape::Shape() {
	_hasCurve = false;
	_hasPolygon = false;
	_hasColumn = false;
}

Shape::~Shape() {
}

Shape Shape::fromXML(const tinyxml2::XMLElement* ele) {
	Shape obj;

	const tinyxml2::XMLElement* subEle;
	subEle = ele->FirstChildElement("curve");
	if (subEle) {
		obj._hasCurve = true;
		obj._curve = shape::Curve::fromXML(subEle);
	}
	subEle = ele->FirstChildElement("polygon");
	if (subEle) {
		obj._hasPolygon = true;
		obj._polygon = shape::Polygon::fromXML(subEle);
	}
	subEle = ele->FirstChildElement("column");
	if (subEle) {
		obj._hasColumn = true;
		obj._column = shape::Column::fromXML(subEle);
	}
	if (!obj.check()) exit(-1);
	return obj;
}

tinyxml2::XMLElement* Shape::toXML(tinyxml2::XMLDocument* xmlDoc, string tagName) const {
	tinyxml2::XMLElement* ele = xmlDoc->NewElement(tagName.c_str());
	if (_hasCurve) ele->InsertEndChild(_curve.toXML(xmlDoc, "curve"));
	if (_hasPolygon) ele->InsertEndChild(_polygon.toXML(xmlDoc, "polygon"));
	if (_hasColumn) ele->InsertEndChild(_column.toXML(xmlDoc, "column"));
	return ele;
}

Shape Shape::fromMsg(const autodrive_msgs::opendrive_object_shape_msg& msg) {
	Shape obj;
	obj._hasCurve = msg.hasCurve;
	if (obj._hasCurve) obj._curve = shape::Curve::fromMsg(msg.curve);
	obj._hasPolygon = msg.hasPolygon;
	if (obj._hasPolygon) obj._polygon = shape::Polygon::fromMsg(msg.polygon);
	obj._hasColumn = msg.hasColumn;
	if (obj._hasColumn) obj._column = shape::Column::fromMsg(msg.column);
	if (!obj.check()) exit(-1);
	return obj;
}

autodrive_msgs::opendrive_object_shape_msg Shape::toMsg() const {
	autodrive_msgs::opendrive_object_shape_msg msg;
	msg.hasCurve = _hasCurve;
	if (_hasCurve) msg.curve = _curve.toMsg();
	msg.hasPolygon = _hasPolygon;
	if (_hasPolygon) msg.polygon = _polygon.toMsg();
	msg.hasColumn = _hasColumn;
	if (_hasColumn) msg.column = _column.toMsg();
	return msg;
}

void Shape::print(string prefix, int level) const {
	if (level < 0) return;
	cout << prefix << "opendrive::object::Shape" << endl;
	cout << prefix << "Subclass: curve" << endl;
	if (_hasCurve) _curve.print(prefix + "  ", level - 1);
	cout << prefix << "Subclass: polygon" << endl;
	if (_hasPolygon) _polygon.print(prefix + "  ", level - 1);
	cout << prefix << "Subclass: column" << endl;
	if (_hasColumn) _column.print(prefix + "  ", level - 1);
	cout << prefix << "--------" << endl;
}

bool Shape::check() const {
	bool status = true;
	if (_hasCurve) status = status && _curve.check();
	if (_hasPolygon) status = status && _polygon.check();
	if (_hasColumn) status = status && _column.check();
	return status;
}

} // namespace object
} // namespace opendrive
