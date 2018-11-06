/********************************************************
*   Copyright (C) 2018 All rights reserved.
*
*   Filename: opendrive_object_shape_color.cpp
*   Author  : lubing.han
*   Date    : 2018-5-16
*   Describe: 
*
********************************************************/
#include "opendrive_object_shape_color.h"
using namespace std;

namespace opendrive {
namespace object {
namespace shape {

Color::Color() {
}

Color::~Color() {
}

Color Color::fromXML(const tinyxml2::XMLElement* ele) {
	Color obj;
	if (!ele->Attribute("r")) {
		cout << "AttributeInfo::getFromXML error: no attribute: r" << endl;
		exit(-1);
	}
	obj._r = atoi(ele->Attribute("r"));
	if (!ele->Attribute("g")) {
		cout << "AttributeInfo::getFromXML error: no attribute: g" << endl;
		exit(-1);
	}
	obj._g = atoi(ele->Attribute("g"));
	if (!ele->Attribute("b")) {
		cout << "AttributeInfo::getFromXML error: no attribute: b" << endl;
		exit(-1);
	}
	obj._b = atoi(ele->Attribute("b"));

	const tinyxml2::XMLElement* subEle;
	if (!obj.check()) exit(-1);
	return obj;
}

tinyxml2::XMLElement* Color::toXML(tinyxml2::XMLDocument* xmlDoc, string tagName) const {
	tinyxml2::XMLElement* ele = xmlDoc->NewElement(tagName.c_str());
	ele->SetAttribute("r", _r);
	ele->SetAttribute("g", _g);
	ele->SetAttribute("b", _b);
	return ele;
}

Color Color::fromMsg(const autodrive_msgs::opendrive_object_shape_color_msg& msg) {
	Color obj;
	obj._r = msg.r;
	obj._g = msg.g;
	obj._b = msg.b;
	if (!obj.check()) exit(-1);
	return obj;
}

autodrive_msgs::opendrive_object_shape_color_msg Color::toMsg() const {
	autodrive_msgs::opendrive_object_shape_color_msg msg;
	msg.r = _r;
	msg.g = _g;
	msg.b = _b;
	return msg;
}

void Color::print(string prefix, int level) const {
	if (level < 0) return;
	cout << prefix << "opendrive::object::shape::Color" << endl;
	cout << prefix << "r: " << (int)_r << endl;
	cout << prefix << "g: " << (int)_g << endl;
	cout << prefix << "b: " << (int)_b << endl;
	cout << prefix << "--------" << endl;
}

bool Color::check() const {
	bool status = true;
	return status;
}

} // namespace shape
} // namespace object
} // namespace opendrive
