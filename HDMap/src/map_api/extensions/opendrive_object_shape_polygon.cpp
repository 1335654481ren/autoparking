/********************************************************
*   Copyright (C) 2018 All rights reserved.
*
*   Filename: opendrive_object_shape_polygon.cpp
*   Author  : lubing.han
*   Date    : 2018-5-16
*   Describe: 
*
********************************************************/
#include "opendrive_object_shape_polygon.h"
using namespace std;

namespace opendrive {
namespace object {
namespace shape {

Polygon::Polygon() {
	_nonconvex = false;
}

Polygon::~Polygon() {
}

Polygon Polygon::fromXML(const tinyxml2::XMLElement* ele) {
	Polygon obj;
	if (ele->Attribute("nonconvex")) {
		obj._nonconvex = (strcmp(ele->Attribute("nonconvex"), "true") == 0);
	}

	const tinyxml2::XMLElement* subEle;
	subEle = ele->FirstChildElement("point");
	while (subEle) {
		obj._point.push_back(Point::fromXML(subEle));
		subEle = subEle->NextSiblingElement("point");
	}
	if (!obj.check()) exit(-1);
	return obj;
}

tinyxml2::XMLElement* Polygon::toXML(tinyxml2::XMLDocument* xmlDoc, string tagName) const {
	tinyxml2::XMLElement* ele = xmlDoc->NewElement(tagName.c_str());
	ele->SetAttribute("nonconvex", _nonconvex);
	for (auto &i : _point) ele->InsertEndChild(i.toXML(xmlDoc, "point"));
	return ele;
}

Polygon Polygon::fromMsg(const autodrive_msgs::opendrive_object_shape_polygon_msg& msg) {
	Polygon obj;
	obj._nonconvex = msg.nonconvex;
	for (auto &i : msg.point) obj._point.push_back(Point::fromMsg(i));
	if (!obj.check()) exit(-1);
	return obj;
}

autodrive_msgs::opendrive_object_shape_polygon_msg Polygon::toMsg() const {
	autodrive_msgs::opendrive_object_shape_polygon_msg msg;
	msg.nonconvex = _nonconvex;
	for (auto &i : _point) msg.point.push_back(i.toMsg());
	return msg;
}

void Polygon::print(string prefix, int level) const {
	if (level < 0) return;
	cout << prefix << "opendrive::object::shape::Polygon" << endl;
	cout << prefix << "nonconvex: " << _nonconvex << endl;
	cout << prefix << "Subclass: point" << endl;
	for (auto &i : _point) i.print(prefix + "  ", level - 1);
	cout << prefix << "--------" << endl;
}

bool Polygon::check() const {
	bool status = true;
	if (!(_point.size() >= 3)) return false;
	for (auto &i : _point) status = status && i.check();
	return status;
}

} // namespace shape
} // namespace object
} // namespace opendrive
