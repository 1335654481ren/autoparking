/********************************************************
*   Copyright (C) 2018 All rights reserved.
*
*   Filename: opendrive_object_shape_column.cpp
*   Author  : lubing.han
*   Date    : 2018-5-16
*   Describe: 
*
********************************************************/
#include "opendrive_object_shape_column.h"
using namespace std;

namespace opendrive {
namespace object {
namespace shape {

Column::Column() {
	_height = 2.0;
	_hasCurve = false;
	_hasPolygon = false;
	_hasVerticalEdge = false;
}

Column::~Column() {
}

Column Column::fromXML(const tinyxml2::XMLElement* ele) {
	Column obj;
	if (ele->Attribute("height")) {
		obj._height = atof(ele->Attribute("height"));
	}

	const tinyxml2::XMLElement* subEle;
	subEle = ele->FirstChildElement("curve");
	if (subEle) {
		obj._hasCurve = true;
		obj._curve = Curve::fromXML(subEle);
	}
	subEle = ele->FirstChildElement("polygon");
	if (subEle) {
		obj._hasPolygon = true;
		obj._polygon = Polygon::fromXML(subEle);
	}
	subEle = ele->FirstChildElement("verticalEdge");
	if (subEle) {
		obj._hasVerticalEdge = true;
		obj._verticalEdge = column::VerticalEdge::fromXML(subEle);
	}
	if (!obj.check()) exit(-1);
	return obj;
}

tinyxml2::XMLElement* Column::toXML(tinyxml2::XMLDocument* xmlDoc, string tagName) const {
	tinyxml2::XMLElement* ele = xmlDoc->NewElement(tagName.c_str());
	ele->SetAttribute("height", _height);
	if (_hasCurve) ele->InsertEndChild(_curve.toXML(xmlDoc, "curve"));
	if (_hasPolygon) ele->InsertEndChild(_polygon.toXML(xmlDoc, "polygon"));
	if (_hasVerticalEdge) ele->InsertEndChild(_verticalEdge.toXML(xmlDoc, "verticalEdge"));
	return ele;
}

Column Column::fromMsg(const autodrive_msgs::opendrive_object_shape_column_msg& msg) {
	Column obj;
	obj._height = msg.height;
	obj._hasCurve = msg.hasCurve;
	if (obj._hasCurve) obj._curve = Curve::fromMsg(msg.curve);
	obj._hasPolygon = msg.hasPolygon;
	if (obj._hasPolygon) obj._polygon = Polygon::fromMsg(msg.polygon);
	obj._hasVerticalEdge = msg.hasVerticalEdge;
	if (obj._hasVerticalEdge) obj._verticalEdge = column::VerticalEdge::fromMsg(msg.verticalEdge);
	if (!obj.check()) exit(-1);
	return obj;
}

autodrive_msgs::opendrive_object_shape_column_msg Column::toMsg() const {
	autodrive_msgs::opendrive_object_shape_column_msg msg;
	msg.height = _height;
	msg.hasCurve = _hasCurve;
	if (_hasCurve) msg.curve = _curve.toMsg();
	msg.hasPolygon = _hasPolygon;
	if (_hasPolygon) msg.polygon = _polygon.toMsg();
	msg.hasVerticalEdge = _hasVerticalEdge;
	if (_hasVerticalEdge) msg.verticalEdge = _verticalEdge.toMsg();
	return msg;
}

void Column::print(string prefix, int level) const {
	if (level < 0) return;
	cout << prefix << "opendrive::object::shape::Column" << endl;
	cout << prefix << "height: " << _height << endl;
	cout << prefix << "Subclass: curve" << endl;
	if (_hasCurve) _curve.print(prefix + "  ", level - 1);
	cout << prefix << "Subclass: polygon" << endl;
	if (_hasPolygon) _polygon.print(prefix + "  ", level - 1);
	cout << prefix << "Subclass: verticalEdge" << endl;
	if (_hasVerticalEdge) _verticalEdge.print(prefix + "  ", level - 1);
	cout << prefix << "--------" << endl;
}

bool Column::check() const {
	bool status = true;
	if (_hasCurve) status = status && _curve.check();
	if (_hasPolygon) status = status && _polygon.check();
	if (_hasVerticalEdge) status = status && _verticalEdge.check();
	return status;
}

} // namespace shape
} // namespace object
} // namespace opendrive
