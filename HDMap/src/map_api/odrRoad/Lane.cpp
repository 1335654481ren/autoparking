/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename:Lane.cpp
*   Author  :weiwei.liu
*   Date    :2016-11-16
*   Describe:
*
********************************************************/

#include "Lane.h"
#include <limits.h>
namespace opendrive{

Polynomial::Polynomial() {
	_offset = 0;
}

Polynomial::Polynomial(double offset, double a, double b, double c, double d) {
	_offset = offset;
	_params.push_back(a);
	_params.push_back(b);
	_params.push_back(c);
	_params.push_back(d);
}

Polynomial::Polynomial(double offset, double a) {
	_offset = offset;
	_params.push_back(a);
}

Polynomial::~Polynomial() {

}

double Polynomial::get_valueof(double x) const {
	double result = 0.0;
	double cur = 1.0;
	for (int i = 0; i < _params.size(); i++ ){
		result += _params[i] * cur;
		cur = cur * ( x - _offset );
	}
	return result;
}

RoadMark::RoadMark() {
	_sOffset = 0.0;
	_type = "none";
	_weight = "standard";
	_color = "standard";
	_material = "standard";
	_width = 0.0;
	_laneChange = "both";
	_height = 0.0;
}

RoadMark::~RoadMark() {

}

bool RoadMark::checkLaneChange(const string& direction) const {
	return _laneChange == "both" || _laneChange == direction;
}

Lane::Lane() {
	_id = 0; 
	_successor = _predecessor = INT_MAX;
}

Lane::Lane(int id) {
	_id = id;
	_successor = _predecessor = INT_MAX;
}

Lane::~Lane() {

}

double Lane::get_valueof(const std::vector<Polynomial>& vec, double s) const {
	s -= _sOffset;
	if (vec.size() == 0) return 0.0;
	int n = vec.size();
	for (int i = 1; i < n; ++i) {
		if (vec[i].get_offset() > s)
			return vec[i - 1].get_valueof(s);
	}
	return vec[n - 1].get_valueof(s);
}

int Lane::get_roadmark_index(double s) const {
	s -= _sOffset;
	if (_roadmarks.size() == 0) return -1;
	int i = 1, n = _roadmarks.size();
	for ( ; i < n; ++i) {
		if (_roadmarks[i].get_sOffset() > s)
			break;
	}
	return i - 1;
}

const RoadMark Lane::get_roadmark(double s) const {
	int i = get_roadmark_index(s);
	return i < 0 ? RoadMark() : _roadmarks[i];
}

void Lane::append_predecessor(const GlobalLaneId &globalid) {
	for (int i = 0; i < _predecessors.size(); ++i)
		if (_predecessors[i] == globalid) return;
	_predecessors.push_back(globalid);
}

void Lane::append_successor(const GlobalLaneId &globalid) {
	for (int i = 0; i < _successors.size(); ++i)
		if (_successors[i] == globalid) return;
	_successors.push_back(globalid);
}

void Lane::append_inner_neighbour(const GlobalLaneId &globalid) {
	for (int i = 0; i < _inner_neighbour.size(); ++i)
		if (_inner_neighbour[i] == globalid) return;
	_inner_neighbour.push_back(globalid);
}

void Lane::append_outer_neighbour(const GlobalLaneId &globalid) {
	for (int i = 0; i < _outer_neighbour.size(); ++i)
		if (_outer_neighbour[i] == globalid) return;
	_outer_neighbour.push_back(globalid);
}

RELATION::RelationType Lane::get_relation(const GlobalLaneId &globalid) const {
	for (int i = 0; i < _predecessors.size(); ++i)
		if (_predecessors[i] == globalid)
			return RELATION::PREDECESSOR;
	for (int i = 0; i < _successors.size(); ++i)
		if (_successors[i] == globalid)
			return RELATION::SUCCESSOR;
	for (int i = 0; i < _inner_neighbour.size(); ++i)
		if (_inner_neighbour[i] == globalid)
			return RELATION::INNER_NEIGHBOUR;
	for (int i = 0; i < _outer_neighbour.size(); ++i)
		if (_outer_neighbour[i] == globalid)
			return RELATION::OUTER_NEIGHBOUR;
	return RELATION::NONE;
}

}