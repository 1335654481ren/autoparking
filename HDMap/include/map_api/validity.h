/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: validity.h
*   Author  : lubing.han
*   Date    : 2016-11-24
*   Describe:
*
********************************************************/
#ifndef MAP_API_VALIDITY_H
#define MAP_API_VALIDITY_H

#include <string>
#include <iostream>
using namespace std;

namespace opendrive
{

class Validity
{
public:
	Validity(): _fromLane(0), _toLane(0) {}
	Validity(int fromLane, int toLane): _fromLane(fromLane), _toLane(toLane) {}
	~Validity() {}
	void print() {cout << "    Validity from: " << _fromLane << " to: " << _toLane << endl;}
	int get_fromLane() const {return _fromLane;}
	int get_toLane() const {return _toLane;}
	void set_fromLane(const int fromLane) {_fromLane = fromLane;}
	void set_toLane(const int toLane) {_toLane = toLane;} 
	bool is_valid(int lane_index) const {return lane_index <= _toLane && lane_index >= _fromLane;}

private:
	int _fromLane;
	int _toLane;
};

}

#endif
