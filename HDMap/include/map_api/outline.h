/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: outline.h
*   Author  : lubing.han
*   Date    : 2016-11-17
*   Describe:
*
********************************************************/
#ifndef MAP_API_OUTLINE_H
#define MAP_API_OUTLINE_H

#include <string>
#include <vector>
#include <iostream>
using namespace std;

namespace opendrive
{

class CornerRoad
{
public:
	CornerRoad():_s(0.0), _t(0.0), _dz(0.0), _height(0.0) {}
	~CornerRoad() {}
	void print() {cout << "      CornerRoad s: " << _s << ", t: " << _t << endl;}
	const double get_s() const {return _s;}
	const double get_t() const {return _t;}
	const double get_dz() const {return _dz;}
	const double get_height() const {return _height;}
	void set_s(const double s) {_s = s;}
	void set_t(const double t) {_t = t;}
	void set_dz(const double dz) {_dz = dz;}
	void set_height(const double height) {_height = height;}
private:
	double _s;
	double _t;
	double _dz;
	double _height;
};

class Outline
{
public:
	Outline() {}
	~Outline() {}
	void print() {
		cout << "    Outline:" << endl;
		for (int i = 0; i < _cornerRoads.size(); ++i)
			_cornerRoads[i].print();
	}
	const vector<CornerRoad>& get_cornerRoads() const {return _cornerRoads;}
	void append_cornerRoad(const CornerRoad &cornerRoad) {_cornerRoads.push_back(cornerRoad);}
private:
	vector<CornerRoad> _cornerRoads;
};

}

#endif
