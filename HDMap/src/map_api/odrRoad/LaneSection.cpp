/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename:LaneSection.cpp
*   Author  :weiwei.liu
*   Date    :2016-11-17
*   Describe:
*
********************************************************/

#include "LaneSection.h"
#include <math.h>
using namespace std;

namespace opendrive {

LaneSection::LaneSection() {
	_singleside = false;
	_sOffset = 0;
}

LaneSection::~LaneSection() {
	_lanes.clear();
}

LaneSection::LaneSection(bool single, double s) {
	_singleside = single;
	_sOffset = s;
}

Lane* LaneSection::operator[](int id) {
	for (int i = 0; i < _lanes.size(); ++i)
		if (_lanes[i].get_id() == id)
			return &(_lanes[i]);
	return NULL;
}

Lane* LaneSection::front() {
	return &_lanes.front();
}

Lane* LaneSection::back() {
	return &_lanes.back();
}

Lane* LaneSection::next(const Lane* lane) {
	int i = 0;
	for ( ; i < _lanes.size(); ++i)
		if (_lanes[i].get_id() == lane->get_id()) break;
	return i + 1 >= _lanes.size() ? NULL : &_lanes[i+1];
}

const Lane* LaneSection::operator[](int id) const {
	for (int i = 0; i < _lanes.size(); ++i)
		if (_lanes[i].get_id() == id)
			return &(_lanes[i]);
	return NULL;
}

const Lane* LaneSection::front() const {
	return &_lanes.front();
}

const Lane* LaneSection::back() const {
	return &_lanes.back();
}

const Lane* LaneSection::next(const Lane* lane) const {
	return get_decrease_lane(lane->get_id());
}

const Lane* LaneSection::get_increase_lane(int id) const {
	int i = (int)_lanes.size() - 1;
	for ( ; i >= 0; --i)
		if (_lanes[i].get_id() == id) break;
	return i <= 0 ? NULL : &_lanes[i -1];

}
const Lane* LaneSection::get_decrease_lane(int id) const {
	int i = 0;
	for ( ; i < _lanes.size(); ++i)
		if (_lanes[i].get_id() == id) break;
	return i + 1 >= _lanes.size() ? NULL : &_lanes[i+1];
}

const Lane* LaneSection::get_inner_neighbour(int id) const {
	const Lane *lane1 = (*this)[id], *lane2;
	if (id == 0 || !lane1) return NULL;
	if (id < 0) lane2 = get_increase_lane(id);
	else lane2 = get_decrease_lane(id);
	if (!lane2 || lane2->get_id() == 0) return NULL;
	return lane2;
	// vector<RoadMark> roadmarks = lane2->get_roadmarks();
	// for (int i = 0; i < roadmarks.size(); ++i)
	// 	if (roadmarks[i].checkLaneChange(id < 0 ? "increase" : "decrease"));
	// 		return lane2;
	// return roadmarks.size() == 0 ? lane2 : NULL;
}

const Lane* LaneSection::get_outer_neighbour(int id) const {
	const Lane *lane1 = (*this)[id], *lane2;
	if (id == 0 || !lane1) return NULL;
	if (id < 0) lane2 = get_decrease_lane(id);
	else lane2 = get_increase_lane(id);
	if (!lane2 || lane2->get_id() == 0) return NULL;
	return lane2;
	// vector<RoadMark> roadmarks = lane1->get_roadmarks();
	// for (int i = 0; i < roadmarks.size(); ++i)
	// 	if (roadmarks[i].checkLaneChange(id < 0 ? "decrease" : "increase"));
	// 		return lane2;
	// return roadmarks.size() == 0 ? lane2 : NULL;
}

vector<double> LaneSection::get_l_by_s(double s) const {
	if (_use_widths) return get_l_by_widths(s);
	else return get_l_by_borders(s);
}

vector<double> LaneSection::get_l_by_widths(double s) const {
	vector<double> ls;
	vector<double> ws;
	for (int i = 0; i < _lanes.size(); ++i) {
		ws.push_back(_lanes[i].get_width(s));
		ls.push_back(0.0);
	}
	double l0 = 0.0;
	for (int i = (int)_lanes.size() - 2; i >= 0; --i) {
		if (_lanes[i].get_id() < 0)
			ls[i] = ls[i + 1] + ws[i + 1];
		else if (_lanes[i].get_id() > 0)
			ls[i] = ls[i + 1] + ws[i];
		else {
			ls[i] = ls[i + 1] + ws[i + 1];
			l0 = ls[i];
		}
	}
	for (int i = 0; i < _lanes.size(); ++i)
		ls[i] -= l0;
	return ls;
}

vector<double> LaneSection::get_l_by_borders(double s) const {
	vector<double> ls;
	for(int i = 0; i < _lanes.size(); i++)
		ls.push_back(_lanes[i].get_border(s));
	return ls;
}

void LaneSection::set_length(double length) {
	_length = length;
	for (int i = 0; i < _lanes.size(); ++i) {
		_lanes[i].set_length(length);
		_lanes[i].set_sOffset(_sOffset);
	}
}

void LaneSection::append_points(const vector<Point> &points) {
	int n = (points.size() < _lanes.size() ? points.size() : _lanes.size());
	for (int i = 0; i < n; ++i) {
		Point lane_point = points[i];
		Point next_point = lane_point;
		if (_lanes[i].get_id() > 0) next_point = points[i + 1];
		else if (_lanes[i].get_id() < 0) next_point = points[i - 1];
		Point center_point = 0.5 * (lane_point + next_point);
		_lanes[i].append_lane_point(lane_point);
		_lanes[i].append_center_point(center_point);
		_lanes[i].append_inner_point(next_point);
	}
}

}