/********************************************************
*   Copyright (C) 2018 All rights reserved.
*
*   Filename: opendrive_object_shape_point_segment.cpp
*   Author  : lubing.han
*   Date    : 2018-5-16
*   Describe: 
*
********************************************************/
#include "opendrive_object_shape_point_segment.h"
using namespace std;

namespace opendrive {
namespace object {
namespace shape {
namespace point {

Segment::Segment() {
	_hasColor = false;
	_hasLeftColor = false;
	_hasRightColor = false;
}

Segment::~Segment() {
}

Segment Segment::fromXML(const tinyxml2::XMLElement* ele) {
	Segment obj;

	const tinyxml2::XMLElement* subEle;
	subEle = ele->FirstChildElement("style");
	if (subEle) {
		obj._style = segment::Style::fromXML(subEle);
	}
	else {
		cout << "ClassContainer::getFromXML error: no subclass: style" << endl;
		exit(-1);
	}
	subEle = ele->FirstChildElement("color");
	if (subEle) {
		obj._hasColor = true;
		obj._color = Color::fromXML(subEle);
	}
	subEle = ele->FirstChildElement("leftColor");
	if (subEle) {
		obj._hasLeftColor = true;
		obj._leftColor = Color::fromXML(subEle);
	}
	subEle = ele->FirstChildElement("rightColor");
	if (subEle) {
		obj._hasRightColor = true;
		obj._rightColor = Color::fromXML(subEle);
	}
	if (!obj.check()) exit(-1);
	return obj;
}

tinyxml2::XMLElement* Segment::toXML(tinyxml2::XMLDocument* xmlDoc, string tagName) const {
	tinyxml2::XMLElement* ele = xmlDoc->NewElement(tagName.c_str());
	ele->InsertEndChild(_style.toXML(xmlDoc, "style"));
	if (_hasColor) ele->InsertEndChild(_color.toXML(xmlDoc, "color"));
	if (_hasLeftColor) ele->InsertEndChild(_leftColor.toXML(xmlDoc, "leftColor"));
	if (_hasRightColor) ele->InsertEndChild(_rightColor.toXML(xmlDoc, "rightColor"));
	return ele;
}

Segment Segment::fromMsg(const autodrive_msgs::opendrive_object_shape_point_segment_msg& msg) {
	Segment obj;
	obj._style = segment::Style::fromMsg(msg.style);
	obj._hasColor = msg.hasColor;
	if (obj._hasColor) obj._color = Color::fromMsg(msg.color);
	obj._hasLeftColor = msg.hasLeftColor;
	if (obj._hasLeftColor) obj._leftColor = Color::fromMsg(msg.leftColor);
	obj._hasRightColor = msg.hasRightColor;
	if (obj._hasRightColor) obj._rightColor = Color::fromMsg(msg.rightColor);
	if (!obj.check()) exit(-1);
	return obj;
}

autodrive_msgs::opendrive_object_shape_point_segment_msg Segment::toMsg() const {
	autodrive_msgs::opendrive_object_shape_point_segment_msg msg;
	msg.style = _style.toMsg();
	msg.hasColor = _hasColor;
	if (_hasColor) msg.color = _color.toMsg();
	msg.hasLeftColor = _hasLeftColor;
	if (_hasLeftColor) msg.leftColor = _leftColor.toMsg();
	msg.hasRightColor = _hasRightColor;
	if (_hasRightColor) msg.rightColor = _rightColor.toMsg();
	return msg;
}

void Segment::print(string prefix, int level) const {
	if (level < 0) return;
	cout << prefix << "opendrive::object::shape::point::Segment" << endl;
	cout << prefix << "Subclass: style" << endl;
	_style.print(prefix + "  ", level - 1);
	cout << prefix << "Subclass: color" << endl;
	if (_hasColor) _color.print(prefix + "  ", level - 1);
	cout << prefix << "Subclass: leftColor" << endl;
	if (_hasLeftColor) _leftColor.print(prefix + "  ", level - 1);
	cout << prefix << "Subclass: rightColor" << endl;
	if (_hasRightColor) _rightColor.print(prefix + "  ", level - 1);
	cout << prefix << "--------" << endl;
}

bool Segment::check() const {
	bool status = true;
	status = status && _style.check();
	if (_hasColor) status = status && _color.check();
	if (_hasLeftColor) status = status && _leftColor.check();
	if (_hasRightColor) status = status && _rightColor.check();
	return status;
}

} // namespace point
} // namespace shape
} // namespace object
} // namespace opendrive
