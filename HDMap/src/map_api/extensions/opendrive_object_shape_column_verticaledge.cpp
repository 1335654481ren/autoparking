/********************************************************
*   Copyright (C) 2018 All rights reserved.
*
*   Filename: opendrive_object_shape_column_verticaledge.cpp
*   Author  : lubing.han
*   Date    : 2018-5-16
*   Describe: 
*
********************************************************/
#include "opendrive_object_shape_column_verticaledge.h"
using namespace std;

namespace opendrive {
namespace object {
namespace shape {
namespace column {

VerticalEdge::VerticalEdge() {
	_hasLowerColor = false;
	_hasHigherColor = false;
}

VerticalEdge::~VerticalEdge() {
}

VerticalEdge VerticalEdge::fromXML(const tinyxml2::XMLElement* ele) {
	VerticalEdge obj;
	if (!ele->Attribute("height")) {
		cout << "AttributeInfo::getFromXML error: no attribute: height" << endl;
		exit(-1);
	}
	obj._height = atof(ele->Attribute("height"));

	const tinyxml2::XMLElement* subEle;
	subEle = ele->FirstChildElement("lowerColor");
	if (subEle) {
		obj._hasLowerColor = true;
		obj._lowerColor = Color::fromXML(subEle);
	}
	subEle = ele->FirstChildElement("higherColor");
	if (subEle) {
		obj._hasHigherColor = true;
		obj._higherColor = Color::fromXML(subEle);
	}
	if (!obj.check()) exit(-1);
	return obj;
}

tinyxml2::XMLElement* VerticalEdge::toXML(tinyxml2::XMLDocument* xmlDoc, string tagName) const {
	tinyxml2::XMLElement* ele = xmlDoc->NewElement(tagName.c_str());
	ele->SetAttribute("height", _height);
	if (_hasLowerColor) ele->InsertEndChild(_lowerColor.toXML(xmlDoc, "lowerColor"));
	if (_hasHigherColor) ele->InsertEndChild(_higherColor.toXML(xmlDoc, "higherColor"));
	return ele;
}

VerticalEdge VerticalEdge::fromMsg(const autodrive_msgs::opendrive_object_shape_column_verticaledge_msg& msg) {
	VerticalEdge obj;
	obj._height = msg.height;
	obj._hasLowerColor = msg.hasLowerColor;
	if (obj._hasLowerColor) obj._lowerColor = Color::fromMsg(msg.lowerColor);
	obj._hasHigherColor = msg.hasHigherColor;
	if (obj._hasHigherColor) obj._higherColor = Color::fromMsg(msg.higherColor);
	if (!obj.check()) exit(-1);
	return obj;
}

autodrive_msgs::opendrive_object_shape_column_verticaledge_msg VerticalEdge::toMsg() const {
	autodrive_msgs::opendrive_object_shape_column_verticaledge_msg msg;
	msg.height = _height;
	msg.hasLowerColor = _hasLowerColor;
	if (_hasLowerColor) msg.lowerColor = _lowerColor.toMsg();
	msg.hasHigherColor = _hasHigherColor;
	if (_hasHigherColor) msg.higherColor = _higherColor.toMsg();
	return msg;
}

void VerticalEdge::print(string prefix, int level) const {
	if (level < 0) return;
	cout << prefix << "opendrive::object::shape::column::VerticalEdge" << endl;
	cout << prefix << "height: " << _height << endl;
	cout << prefix << "Subclass: lowerColor" << endl;
	if (_hasLowerColor) _lowerColor.print(prefix + "  ", level - 1);
	cout << prefix << "Subclass: higherColor" << endl;
	if (_hasHigherColor) _higherColor.print(prefix + "  ", level - 1);
	cout << prefix << "--------" << endl;
}

bool VerticalEdge::check() const {
	bool status = true;
	if (_hasLowerColor) status = status && _lowerColor.check();
	if (_hasHigherColor) status = status && _higherColor.check();
	return status;
}

} // namespace column
} // namespace shape
} // namespace object
} // namespace opendrive
