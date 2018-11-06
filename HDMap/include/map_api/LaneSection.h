/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename:LaneSection.h
*   Author  :weiwei.liu
*   Date    :2016-11-17
*   Describe:
*
********************************************************/

#ifndef HOBOT_LANESECTION_H
#define HOBOT_LANESECTION_H

#include "Lane.h"
#include "map_api_parameters.h"
#include <vector>
#include <iostream>
using namespace std;

namespace opendrive{

class LaneSection {
public:
	LaneSection();
	LaneSection(bool single, double s);
	~LaneSection();

	Lane* operator[](int id);
	Lane* front();
	Lane* back();
	Lane* next(const Lane* lane);
	const Lane* operator[](int id) const;
	const Lane* front() const;
	const Lane* back() const;
	const Lane* next(const Lane* lane) const;
	const Lane* get_increase_lane(int id) const;
	const Lane* get_decrease_lane(int id) const;
	const Lane* get_inner_neighbour(int id) const;
	const Lane* get_outer_neighbour(int id) const;

	int get_id() const {return _id;}
	bool get_singleside() const {return _singleside;}
	bool get_use_widths() const {return _use_widths;}
	double get_sOffset() const {return _sOffset;}
	double get_length() const {return _length;}
	vector<Lane>& get_lanes() {return _lanes;}
	int get_lane_id(int index) const {return _lanes[index].get_id();}

	void set_id(int id) {_id = id;}
	void set_singleside(bool single) {_singleside = single;}
	void set_use_widths(bool use_widths) {_use_widths = use_widths;}
	void set_sOffset(double s) {_sOffset = s;}
	void set_length(double length);
	void append_lane(const Lane& lane) {_lanes.push_back(lane);}
	void append_points(const vector<Point> &points);
	vector<double> get_l_by_s(double s) const;

private:
	vector<double> get_l_by_widths(double s) const;
	vector<double> get_l_by_borders(double s) const;

private:
	vector<Lane> _lanes;
	int _id;
	bool _singleside;
	bool _use_widths = false;
	double _sOffset;
	double _length;
};

}


#endif


