/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: dependency.h
*   Author  : lubing.han
*   Date    : 2016-11-17
*   Describe:
*
********************************************************/
#ifndef MAP_API_DEPENDENCY_H
#define MAP_API_DEPENDENCY_H

#include <string>
#include <iostream>
using namespace std;

namespace opendrive
{

class Dependency
{
public:
	Dependency(): _id(""), _type("") {}
	Dependency(string id, string type): _id(id), _type(type) {}
	~Dependency() {}
	void print() {cout << "    Denpendency id: " << _id << ", type: " << _type << endl;}
	string get_id() const {return _id;}
	string get_type() const {return _type;}
	void set_id(const string id) {_id = id;}
	void set_type(const string type) {_type = type;}
private:
	string _id;
	string _type;
};

}

#endif