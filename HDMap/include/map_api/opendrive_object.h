/********************************************************
*   Copyright (C) 2018 All rights reserved.
*
*   Filename: opendrive_object.h
*   Author  : lubing.han
*   Date    : 2018-5-16
*   Describe: 
*
********************************************************/
#ifndef OPENDRIVE_OBJECT_H
#define OPENDRIVE_OBJECT_H

#include "tinyxml2.h"
#include "opendrive_object_shape.h"
#ifndef DEROS
#include <autodrive_msgs/opendrive_object_msg.h>
#else
#include "autodrive_msgs_opendrive_object_msg.h"
#endif
#include <vector>
#include <iostream>
using namespace std;

namespace opendrive {

class Object
{
public:
	Object();
	~Object();

	string& getId() {return _id;}
	const string& getId() const {return _id;}
	void setId(const string &id) {_id = id;}

	string& getType() {return _type;}
	const string& getType() const {return _type;}
	void setType(const string &type) {_type = type;}

	bool hasSubtype() const {return _hasSubtype;}
	void setHasSubtype(bool hasSubtype) {_hasSubtype = hasSubtype;}
	string& getSubtype() {return _subtype;}
	const string& getSubtype() const {return _subtype;}
	void setSubtype(const string &subtype) {_subtype = subtype;}

	vector<object::Shape>& getShape() {return _shape;}
	const vector<object::Shape>& getShape() const {return _shape;}
	void setShape(const vector<object::Shape> &shape) {_shape = shape;}
	void appendShape(const object::Shape &shape) {_shape.push_back(shape);}

	static Object fromXML(const tinyxml2::XMLElement* ele);
	tinyxml2::XMLElement* toXML(tinyxml2::XMLDocument* xmlDoc, string tagName) const;
	static Object fromMsg(const autodrive_msgs::opendrive_object_msg& msg);
	autodrive_msgs::opendrive_object_msg toMsg() const;
	void print(string prefix = "", int level = 10) const;
	bool check() const;
private:
	string _id;
	string _type;
	bool _hasSubtype;
	string _subtype;
	vector<object::Shape> _shape;
};
} // namespace opendrive

#endif
