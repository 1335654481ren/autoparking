/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: marker.cpp
*   Author  : lubing.han
*   Date    : 2016-12-26
*   Describe:
*
********************************************************/

#include "marker.h"
using namespace std;

namespace opendrive
{

Marker::Marker() {
	_s = -1.0;
	_t = 0.0;
	_id = "-1";
	_zOffset = 0.0;
	_type = "";
	_name = "";
}

void Marker::print() {
	cout << "  Marker id: " << _id << ", s: " << _s << ", t: " << _t << ", type: " << _type << endl;
	_outline.print();
	for (int i = 0; i < _validities.size(); ++i)
		_validities[i].print();
	for (int i = 0; i < _references.size(); ++i)
		_references[i].print();
}

bool Marker::is_valid(int lane_index) const {
	for (int i = 0; i < _validities.size(); i++) {
		if (_validities[i].is_valid(lane_index)) return true;
	}
	return false;
}

string Marker::get_refId(int laneId) const {
	for (int i = 0; i < _references.size(); i++) {
	   if (_references[i].get_laneId() == laneId) 
		   return _references[i].get_signalId();
	}
	return "";
}


}
