/********************************************************
*   Copyright (C) 2018 All rights reserved.
*
*   Filename: opendrive_object_shape_column_verticaledge.h
*   Author  : lubing.han
*   Date    : 2018-5-16
*   Describe: 
*
********************************************************/
#ifndef OPENDRIVE_OBJECT_SHAPE_COLUMN_VERTICALEDGE_H
#define OPENDRIVE_OBJECT_SHAPE_COLUMN_VERTICALEDGE_H

#include "tinyxml2.h"
#include "opendrive_object_shape_color.h"
#include "opendrive_object_shape_color.h"
#ifndef DEROS
#include <autodrive_msgs/opendrive_object_shape_column_verticaledge_msg.h>
#else
#include "autodrive_msgs_opendrive_object_shape_column_verticaledge_msg.h"
#endif
#include <vector>
#include <iostream>
using namespace std;

namespace opendrive {
namespace object {
namespace shape {
namespace column {

class VerticalEdge
{
public:
	VerticalEdge();
	~VerticalEdge();

	double& getHeight() {return _height;}
	const double& getHeight() const {return _height;}
	void setHeight(const double &height) {_height = height;}

	bool hasLowerColor() const {return _hasLowerColor;}
	void setHasLowerColor(bool hasLowerColor) {_hasLowerColor = hasLowerColor;}
	Color& getLowerColor() {return _lowerColor;}
	const Color& getLowerColor() const {return _lowerColor;}
	void setLowerColor(const Color &lowerColor) {_lowerColor = lowerColor;}

	bool hasHigherColor() const {return _hasHigherColor;}
	void setHasHigherColor(bool hasHigherColor) {_hasHigherColor = hasHigherColor;}
	Color& getHigherColor() {return _higherColor;}
	const Color& getHigherColor() const {return _higherColor;}
	void setHigherColor(const Color &higherColor) {_higherColor = higherColor;}

	static VerticalEdge fromXML(const tinyxml2::XMLElement* ele);
	tinyxml2::XMLElement* toXML(tinyxml2::XMLDocument* xmlDoc, string tagName) const;
	static VerticalEdge fromMsg(const autodrive_msgs::opendrive_object_shape_column_verticaledge_msg& msg);
	autodrive_msgs::opendrive_object_shape_column_verticaledge_msg toMsg() const;
	void print(string prefix = "", int level = 10) const;
	bool check() const;
private:
	double _height;
	bool _hasLowerColor;
	Color _lowerColor;
	bool _hasHigherColor;
	Color _higherColor;
};
} // namespace column
} // namespace shape
} // namespace object
} // namespace opendrive

#endif
