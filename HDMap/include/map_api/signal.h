/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: signal.h
*   Author  : lubing.han
*   Date    : 2016-11-17
*   Describe:
*
********************************************************/
#ifndef MAP_API_SIGNAL_H
#define MAP_API_SIGNAL_H

#include <vector>
#include "validity.h"
#include "dependency.h"
#include <string>
using namespace std;

namespace opendrive
{

class Signal
{
public:
	Signal();
	~Signal() {};
	void print();
	bool   get_isReference() const {return _isReference;}
	double get_s() const {return _s;}
	double get_t() const {return _t;}
	double get_dx() const {return _dx;}
	double get_dy() const {return _dy;}
	string get_id() const {return _id;}
	string get_name() const {return _name;}
	string get_dynamic() const {return _dynamic;}
	string get_orientation() const {return _orientation;}
	double get_zOffset() const {return _zOffset;}
	string get_country() const {return _country;}
	string get_type() const {return _type;}
	string get_subtype() const {return _subtype;}
	double get_value() const {return _value;}
	string get_unit() const {return _unit;}
	double get_height() const {return _height;}
	double get_width() const {return _width;}
	string get_text() const {return _text;}
	double get_hOffset() const {return _hOffset;}
	double get_pitch() const {return _pitch;}
	double get_roll() const {return _roll;}
	const vector<Validity>& get_validities() const {return _validities;}
	const vector<Dependency>& get_dependencies() const {return _dependencies;}
	void set_isReference(const bool isReference) {_isReference = isReference;}
	void set_s(const double s) {_s = s;}
	void set_t(const double t) {_t = t;}
	void set_dx(const double dx) {_dx = dx;}
	void set_dy(const double dy) {_dy = dy;}
	void set_id(const string id) {_id = id;}
	void set_name(const string name) {_name = name;}
	void set_dynamic(const string dynamic) {_dynamic = dynamic;}
	void set_orientation(const string orientation) {_orientation = orientation;}
	void set_zOffset(const double zOffset) {_zOffset = zOffset;}
	void set_country(const string country) {_country = country;}
	void set_type(const string type) {_type = type;}
	void set_subtype(const string subtype) {_subtype = subtype;}
	void set_value(const double value) {_value = value;}
	void set_unit(const string unit) {_unit = unit;}
	void set_height(const double height) {_height = height;}
	void set_width(const double width) {_width = width;}
	void set_text(const string text) {_text = text;}
	void set_hOffset(const double hOffset) {_hOffset = hOffset;}
	void set_pitch(const double pitch) {_pitch = pitch;}
	void set_roll(const double roll) {_roll = roll;}
	void append_validity(const Validity &validity) {_validities.push_back(validity);}
	void set_validities(const vector<Validity> &validities) {_validities = validities;}
	void append_dependency(const Dependency &dependency) {_dependencies.push_back(dependency);}
private:
	bool   _isReference;
	double _s;
	double _t;
	double _dx;
	double _dy;
	string _id;
	string _name;
	string _dynamic;
	string _orientation;
	double _zOffset;
	string _country;
	string _type;
	string _subtype;
	double _value;
	string _unit;
	double _height;
	double _width;
	string _text;
	double _hOffset;
	double _pitch;
	double _roll;
	vector<Validity> _validities;
	vector<Dependency> _dependencies;
};

}

#endif
