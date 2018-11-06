/********************************************************
*   Copyright (C) 2018 All rights reserved.
*
*   Filename: opendrive_object_shape_color.h
*   Author  : lubing.han
*   Date    : 2018-5-16
*   Describe: 
*
********************************************************/
#ifndef OPENDRIVE_OBJECT_SHAPE_COLOR_H
#define OPENDRIVE_OBJECT_SHAPE_COLOR_H

#include "tinyxml2.h"
#ifndef DEROS
#include <autodrive_msgs/opendrive_object_shape_color_msg.h>
#else
#include "autodrive_msgs_opendrive_object_shape_color_msg.h"
#endif
#include <vector>
#include <iostream>
using namespace std;

namespace opendrive {
namespace object {
namespace shape {

class Color
{
public:
	Color();
	~Color();

	unsigned char& getR() {return _r;}
	const unsigned char& getR() const {return _r;}
	void setR(const unsigned char &r) {_r = r;}

	unsigned char& getG() {return _g;}
	const unsigned char& getG() const {return _g;}
	void setG(const unsigned char &g) {_g = g;}

	unsigned char& getB() {return _b;}
	const unsigned char& getB() const {return _b;}
	void setB(const unsigned char &b) {_b = b;}

	static Color fromXML(const tinyxml2::XMLElement* ele);
	tinyxml2::XMLElement* toXML(tinyxml2::XMLDocument* xmlDoc, string tagName) const;
	static Color fromMsg(const autodrive_msgs::opendrive_object_shape_color_msg& msg);
	autodrive_msgs::opendrive_object_shape_color_msg toMsg() const;
	void print(string prefix = "", int level = 10) const;
	bool check() const;
private:
	unsigned char _r;
	unsigned char _g;
	unsigned char _b;
};
} // namespace shape
} // namespace object
} // namespace opendrive

#endif
