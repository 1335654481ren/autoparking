/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: marker.h
*   Author  : lubing.han
*   Date    : 2016-11-24
*   Describe:
*
********************************************************/
#ifndef MAP_API_MARKER_H
#define MAP_API_MARKER_H

#include <string>
#include <vector>
#include "outline.h"
#include "validity.h"
#include "reference.h"
using namespace std;

namespace opendrive
{

class Marker
{
public:
	Marker();
	~Marker() {};
	void print();
	string get_type() const {return _type;}
	string get_name() const {return _name;}
	string get_id() const {return _id;}
	double get_height() const {return _height;}
	double get_s() const {return _s;}
	double get_t() const {return _t;}
	double get_zOffset() const {return _zOffset;}
	const Outline& get_outline() const {return _outline;}
	const vector<Validity>& get_validities() const {return _validities;}
	const vector<Reference>& get_references() const {return _references;}
	void set_type(const string type) {_type = type;}
	void set_name(const string name) {_name = name;}
	void set_id(const string id) {_id = id;}
	void set_height(const double height) {_height = height;}
	void set_s(const double s) {_s = s;}
	void set_t(const double t) {_t = t;}
	void set_zOffset(const double zOffset) {_zOffset = zOffset;}
	void set_outline(const Outline &outline) {_outline = outline;}
	void append_validity(const Validity &validity) {_validities.push_back(validity);}
	void append_reference(const Reference &reference) {_references.push_back(reference);}
	bool is_valid(int lane_index) const;
	string get_refId(int lane_id) const;

private:
	string _type;
	string _name;
	string _id;
	double _s;
	double _t;
	double _zOffset;
	double _height;
	string _nextRoad;
	Outline _outline;
	vector<Validity> _validities;
	vector<Reference> _references;
};

}

#endif
