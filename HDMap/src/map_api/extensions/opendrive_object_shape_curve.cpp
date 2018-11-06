/********************************************************
*   Copyright (C) 2018 All rights reserved.
*
*   Filename: opendrive_object_shape_curve.cpp
*   Author  : lubing.han
*   Date    : 2018-5-16
*   Describe: 
*
********************************************************/
#include "opendrive_object_shape_curve.h"
using namespace std;

namespace opendrive {
namespace object {
namespace shape {

Curve::Curve() {
}

Curve::~Curve() {
}

Curve Curve::fromXML(const tinyxml2::XMLElement* ele) {
	Curve obj;

	const tinyxml2::XMLElement* subEle;
	subEle = ele->FirstChildElement("point");
	while (subEle) {
		obj._point.push_back(Point::fromXML(subEle));
		subEle = subEle->NextSiblingElement("point");
	}
	if (!obj.check()) exit(-1);
	return obj;
}

tinyxml2::XMLElement* Curve::toXML(tinyxml2::XMLDocument* xmlDoc, string tagName) const {
	tinyxml2::XMLElement* ele = xmlDoc->NewElement(tagName.c_str());
	for (auto &i : _point) ele->InsertEndChild(i.toXML(xmlDoc, "point"));
	return ele;
}

Curve Curve::fromMsg(const autodrive_msgs::opendrive_object_shape_curve_msg& msg) {
	Curve obj;
	for (auto &i : msg.point) obj._point.push_back(Point::fromMsg(i));
	if (!obj.check()) exit(-1);
	return obj;
}

autodrive_msgs::opendrive_object_shape_curve_msg Curve::toMsg() const {
	autodrive_msgs::opendrive_object_shape_curve_msg msg;
	for (auto &i : _point) msg.point.push_back(i.toMsg());
	return msg;
}

void Curve::print(string prefix, int level) const {
	if (level < 0) return;
	cout << prefix << "opendrive::object::shape::Curve" << endl;
	cout << prefix << "Subclass: point" << endl;
	for (auto &i : _point) i.print(prefix + "  ", level - 1);
	cout << prefix << "--------" << endl;
}

bool Curve::check() const {
	bool status = true;
	if (!(_point.size() >= 2)) return false;
	for (auto &i : _point) status = status && i.check();
	return status;
}

} // namespace shape
} // namespace object
} // namespace opendrive
