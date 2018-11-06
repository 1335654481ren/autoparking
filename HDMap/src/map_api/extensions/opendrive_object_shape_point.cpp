/********************************************************
*   Copyright (C) 2018 All rights reserved.
*
*   Filename: opendrive_object_shape_point.cpp
*   Author  : lubing.han
*   Date    : 2018-5-16
*   Describe: 
*
********************************************************/
#include "opendrive_object_shape_point.h"
using namespace std;

namespace opendrive {
namespace object {
namespace shape {

Point::Point() {
	_z = 0.0;
	_exact = false;
	_hasSegment = false;
}

Point::~Point() {
}

Point Point::fromXML(const tinyxml2::XMLElement* ele) {
	Point obj;
	if (!ele->Attribute("x")) {
		cout << "AttributeInfo::getFromXML error: no attribute: x" << endl;
		exit(-1);
	}
	obj._x = atof(ele->Attribute("x"));
	if (!ele->Attribute("y")) {
		cout << "AttributeInfo::getFromXML error: no attribute: y" << endl;
		exit(-1);
	}
	obj._y = atof(ele->Attribute("y"));
	if (ele->Attribute("z")) {
		obj._z = atof(ele->Attribute("z"));
	}
	if (ele->Attribute("exact")) {
		obj._exact = (strcmp(ele->Attribute("exact"), "true") == 0);
	}

	const tinyxml2::XMLElement* subEle;
	subEle = ele->FirstChildElement("segment");
	if (subEle) {
		obj._hasSegment = true;
		obj._segment = point::Segment::fromXML(subEle);
	}
	if (!obj.check()) exit(-1);
	return obj;
}

tinyxml2::XMLElement* Point::toXML(tinyxml2::XMLDocument* xmlDoc, string tagName) const {
	tinyxml2::XMLElement* ele = xmlDoc->NewElement(tagName.c_str());
	ele->SetAttribute("x", _x);
	ele->SetAttribute("y", _y);
	ele->SetAttribute("z", _z);
	ele->SetAttribute("exact", _exact);
	if (_hasSegment) ele->InsertEndChild(_segment.toXML(xmlDoc, "segment"));
	return ele;
}

Point Point::fromMsg(const autodrive_msgs::opendrive_object_shape_point_msg& msg) {
	Point obj;
	obj._x = msg.x;
	obj._y = msg.y;
	obj._z = msg.z;
	obj._exact = msg.exact;
	obj._hasSegment = msg.hasSegment;
	if (obj._hasSegment) obj._segment = point::Segment::fromMsg(msg.segment);
	if (!obj.check()) exit(-1);
	return obj;
}

autodrive_msgs::opendrive_object_shape_point_msg Point::toMsg() const {
	autodrive_msgs::opendrive_object_shape_point_msg msg;
	msg.x = _x;
	msg.y = _y;
	msg.z = _z;
	msg.exact = _exact;
	msg.hasSegment = _hasSegment;
	if (_hasSegment) msg.segment = _segment.toMsg();
	return msg;
}

void Point::print(string prefix, int level) const {
	if (level < 0) return;
	cout << prefix << "opendrive::object::shape::Point" << endl;
	cout << prefix << "x: " << _x << endl;
	cout << prefix << "y: " << _y << endl;
	cout << prefix << "z: " << _z << endl;
	cout << prefix << "exact: " << _exact << endl;
	cout << prefix << "Subclass: segment" << endl;
	if (_hasSegment) _segment.print(prefix + "  ", level - 1);
	cout << prefix << "--------" << endl;
}

bool Point::check() const {
	bool status = true;
	if (_hasSegment) status = status && _segment.check();
	return status;
}

} // namespace shape
} // namespace object
} // namespace opendrive
