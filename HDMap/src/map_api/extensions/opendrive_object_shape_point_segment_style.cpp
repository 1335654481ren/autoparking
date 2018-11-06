/********************************************************
*   Copyright (C) 2018 All rights reserved.
*
*   Filename: opendrive_object_shape_point_segment_style.cpp
*   Author  : lubing.han
*   Date    : 2018-5-16
*   Describe: 
*
********************************************************/
#include "opendrive_object_shape_point_segment_style.h"
using namespace std;

namespace opendrive {
namespace object {
namespace shape {
namespace point {
namespace segment {

Style::Style() {
	_type = "solid";
	_width = 0.0;
}

Style::~Style() {
}

Style Style::fromXML(const tinyxml2::XMLElement* ele) {
	Style obj;
	if (ele->Attribute("type")) {
		obj._type = ele->Attribute("type");
	}
	if (ele->Attribute("width")) {
		obj._width = atof(ele->Attribute("width"));
	}

	const tinyxml2::XMLElement* subEle;
	if (!obj.check()) exit(-1);
	return obj;
}

tinyxml2::XMLElement* Style::toXML(tinyxml2::XMLDocument* xmlDoc, string tagName) const {
	tinyxml2::XMLElement* ele = xmlDoc->NewElement(tagName.c_str());
	ele->SetAttribute("type", _type.c_str());
	ele->SetAttribute("width", _width);
	return ele;
}

Style Style::fromMsg(const autodrive_msgs::opendrive_object_shape_point_segment_style_msg& msg) {
	Style obj;
	obj._type = msg.type;
	obj._width = msg.width;
	if (!obj.check()) exit(-1);
	return obj;
}

autodrive_msgs::opendrive_object_shape_point_segment_style_msg Style::toMsg() const {
	autodrive_msgs::opendrive_object_shape_point_segment_style_msg msg;
	msg.type = _type;
	msg.width = _width;
	return msg;
}

void Style::print(string prefix, int level) const {
	if (level < 0) return;
	cout << prefix << "opendrive::object::shape::point::segment::Style" << endl;
	cout << prefix << "type: " << _type << endl;
	cout << prefix << "width: " << _width << endl;
	cout << prefix << "--------" << endl;
}

bool Style::check() const {
	bool status = true;
	return status;
}

} // namespace segment
} // namespace point
} // namespace shape
} // namespace object
} // namespace opendrive
