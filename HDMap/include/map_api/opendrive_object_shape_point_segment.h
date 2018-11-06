/********************************************************
*   Copyright (C) 2018 All rights reserved.
*
*   Filename: opendrive_object_shape_point_segment.h
*   Author  : lubing.han
*   Date    : 2018-5-16
*   Describe: 
*
********************************************************/
#ifndef OPENDRIVE_OBJECT_SHAPE_POINT_SEGMENT_H
#define OPENDRIVE_OBJECT_SHAPE_POINT_SEGMENT_H

#include "tinyxml2.h"
#include "opendrive_object_shape_point_segment_style.h"
#include "opendrive_object_shape_color.h"
#include "opendrive_object_shape_color.h"
#include "opendrive_object_shape_color.h"
#ifndef DEROS
#include <autodrive_msgs/opendrive_object_shape_point_segment_msg.h>
#else
#include "autodrive_msgs_opendrive_object_shape_point_segment_msg.h"
#endif
#include <vector>
#include <iostream>
using namespace std;

namespace opendrive {
namespace object {
namespace shape {
namespace point {

class Segment
{
public:
	Segment();
	~Segment();

	segment::Style& getStyle() {return _style;}
	const segment::Style& getStyle() const {return _style;}
	void setStyle(const segment::Style &style) {_style = style;}

	bool hasColor() const {return _hasColor;}
	void setHasColor(bool hasColor) {_hasColor = hasColor;}
	Color& getColor() {return _color;}
	const Color& getColor() const {return _color;}
	void setColor(const Color &color) {_color = color;}

	bool hasLeftColor() const {return _hasLeftColor;}
	void setHasLeftColor(bool hasLeftColor) {_hasLeftColor = hasLeftColor;}
	Color& getLeftColor() {return _leftColor;}
	const Color& getLeftColor() const {return _leftColor;}
	void setLeftColor(const Color &leftColor) {_leftColor = leftColor;}

	bool hasRightColor() const {return _hasRightColor;}
	void setHasRightColor(bool hasRightColor) {_hasRightColor = hasRightColor;}
	Color& getRightColor() {return _rightColor;}
	const Color& getRightColor() const {return _rightColor;}
	void setRightColor(const Color &rightColor) {_rightColor = rightColor;}

	static Segment fromXML(const tinyxml2::XMLElement* ele);
	tinyxml2::XMLElement* toXML(tinyxml2::XMLDocument* xmlDoc, string tagName) const;
	static Segment fromMsg(const autodrive_msgs::opendrive_object_shape_point_segment_msg& msg);
	autodrive_msgs::opendrive_object_shape_point_segment_msg toMsg() const;
	void print(string prefix = "", int level = 10) const;
	bool check() const;
private:
	segment::Style _style;
	bool _hasColor;
	Color _color;
	bool _hasLeftColor;
	Color _leftColor;
	bool _hasRightColor;
	Color _rightColor;
};
} // namespace point
} // namespace shape
} // namespace object
} // namespace opendrive

#endif
