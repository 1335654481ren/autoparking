/********************************************************
*   Copyright (C) 2018 All rights reserved.
*
*   Filename: opendrive_object.cpp
*   Author  : lubing.han
*   Date    : 2018-5-16
*   Describe: 
*
********************************************************/
#include "opendrive_object.h"
using namespace std;

namespace opendrive {

Object::Object() {
	_hasSubtype = false;
}

Object::~Object() {
}

Object Object::fromXML(const tinyxml2::XMLElement* ele) {
	Object obj;
	if (!ele->Attribute("id")) {
		cout << "AttributeInfo::getFromXML error: no attribute: id" << endl;
		exit(-1);
	}
	obj._id = ele->Attribute("id");
	if (!ele->Attribute("type")) {
		cout << "AttributeInfo::getFromXML error: no attribute: type" << endl;
		exit(-1);
	}
	obj._type = ele->Attribute("type");
	if (ele->Attribute("subtype")) {
		obj._hasSubtype = true;
		obj._subtype = ele->Attribute("subtype");
	}

	const tinyxml2::XMLElement* subEle;
	subEle = ele->FirstChildElement("shape");
	while (subEle) {
		obj._shape.push_back(object::Shape::fromXML(subEle));
		subEle = subEle->NextSiblingElement("shape");
	}
	if (!obj.check()) exit(-1);
	return obj;
}

tinyxml2::XMLElement* Object::toXML(tinyxml2::XMLDocument* xmlDoc, string tagName) const {
	tinyxml2::XMLElement* ele = xmlDoc->NewElement(tagName.c_str());
	ele->SetAttribute("id", _id.c_str());
	ele->SetAttribute("type", _type.c_str());
	if (_hasSubtype) ele->SetAttribute("subtype", _subtype.c_str());
	for (auto &i : _shape) ele->InsertEndChild(i.toXML(xmlDoc, "shape"));
	return ele;
}

Object Object::fromMsg(const autodrive_msgs::opendrive_object_msg& msg) {
	Object obj;
	obj._id = msg.id;
	obj._type = msg.type;
	obj._hasSubtype = msg.hasSubtype;
	if (obj._hasSubtype) obj._subtype = msg.subtype;
	for (auto &i : msg.shape) obj._shape.push_back(object::Shape::fromMsg(i));
	if (!obj.check()) exit(-1);
	return obj;
}

autodrive_msgs::opendrive_object_msg Object::toMsg() const {
	autodrive_msgs::opendrive_object_msg msg;
	msg.id = _id;
	msg.type = _type;
	msg.hasSubtype = _hasSubtype;
	if (_hasSubtype) msg.subtype = _subtype;
	for (auto &i : _shape) msg.shape.push_back(i.toMsg());
	return msg;
}

void Object::print(string prefix, int level) const {
	if (level < 0) return;
	cout << prefix << "opendrive::Object" << endl;
	cout << prefix << "id: " << _id << endl;
	cout << prefix << "type: " << _type << endl;
	if (_hasSubtype) cout << prefix << "subtype: " << _subtype << endl;
	cout << prefix << "Subclass: shape" << endl;
	for (auto &i : _shape) i.print(prefix + "  ", level - 1);
	cout << prefix << "--------" << endl;
}

bool Object::check() const {
	bool status = true;
	if (!(_shape.size() >= 0)) return false;
	for (auto &i : _shape) status = status && i.check();
	return status;
}

} // namespace opendrive
