/********************************************************
*   Copyright (C) 2018 All rights reserved.
*
*   Filename: opendrive_object_shape_point_segment_style.h
*   Author  : lubing.han
*   Date    : 2018-5-16
*   Describe: 
*
********************************************************/
#ifndef OPENDRIVE_OBJECT_SHAPE_POINT_SEGMENT_STYLE_H
#define OPENDRIVE_OBJECT_SHAPE_POINT_SEGMENT_STYLE_H

#include "tinyxml2.h"
#ifndef DEROS
#include <autodrive_msgs/opendrive_object_shape_point_segment_style_msg.h>
#else
#include "autodrive_msgs_opendrive_object_shape_point_segment_style_msg.h"
#endif
#include <vector>
#include <iostream>
using namespace std;

namespace opendrive {
namespace object {
namespace shape {
namespace point {
namespace segment {

class Style
{
public:
	Style();
	~Style();

	string& getType() {return _type;}
	const string& getType() const {return _type;}
	void setType(const string &type) {_type = type;}

	double& getWidth() {return _width;}
	const double& getWidth() const {return _width;}
	void setWidth(const double &width) {_width = width;}

	static Style fromXML(const tinyxml2::XMLElement* ele);
	tinyxml2::XMLElement* toXML(tinyxml2::XMLDocument* xmlDoc, string tagName) const;
	static Style fromMsg(const autodrive_msgs::opendrive_object_shape_point_segment_style_msg& msg);
	autodrive_msgs::opendrive_object_shape_point_segment_style_msg toMsg() const;
	void print(string prefix = "", int level = 10) const;
	bool check() const;
private:
	string _type;
	double _width;
};
} // namespace segment
} // namespace point
} // namespace shape
} // namespace object
} // namespace opendrive

#endif
