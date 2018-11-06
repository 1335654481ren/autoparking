/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: signal.cpp
*   Author  : lubing.han
*   Date    : 2016-12-23
*   Describe:
*
********************************************************/

#include "signal.h"
using namespace std;

namespace opendrive
{

Signal::Signal() {
	_isReference = false;
	_s = -1.0;
	_t = 0.0;
	_dx = 0.0;
	_dy = 0.0;
	_id = "-1";
	_name = "";
	_dynamic = "no";
	_orientation = "none";
	_zOffset = 0.0;
	_country = "";
	_type = "";
	_subtype ="";
	_value = 0.0;
	_unit = "";
	_height = 0.0;
	_width = 0.0;
	_text = "";
	_hOffset = 0.0;
	_pitch = 0.0;
	_roll = 0.0;
}

void Signal::print() {
	if (_isReference)
		cout << "  Signal s: " << _s << ", t: " << _t << ", reference id: " << _id << endl;
	else {
		cout << "  Signal s: " << _s << ", t: " << _t << ", id: " << _id << endl;
		cout << "         orientation: " << _orientation << ", type: " << _type << ", subtype; " << _subtype << endl;
	}
	for (int i = 0; i < _validities.size(); ++i)
		_validities[i].print();
	for (int i = 0; i < _dependencies.size(); ++i)
		_dependencies[i].print();
}

}