/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: reference.h
*   Author  : lubing.han
*   Date    : 2016-12-27
*   Describe:
*
********************************************************/
#ifndef MAP_API_REFERENCE_H
#define MAP_API_REFERENCE_H

#include <string>
#include <iostream>
using namespace std;

namespace opendrive
{

class Reference
{
public:
	Reference(): _signalId(""), _laneId(0) {}
	Reference(string signalId, int laneId): _signalId(signalId), _laneId(laneId) {}
	~Reference() {}
	void print() {cout << "    Referenced signal id: " << _signalId << ", lane id: " << _laneId << endl;}
	string get_signalId() const {return _signalId;}
	int get_laneId() const {return _laneId;}
	void set_signalId(const string signalId) {_signalId = signalId;}
	void set_laneId(const int laneId) {_laneId = laneId;}
private:
	string _signalId;
	int _laneId;
};

}

#endif
