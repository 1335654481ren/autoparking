/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename:road.cpp
*   Author  :weiwei.liu
*   Date    :2016-11-17
*   Describe:
*
********************************************************/

#include "road.h"
#include <math.h>
#include <algorithm>

namespace opendrive {

LaneSection* Road::operator[](int i) {
	if (i < 0 || i >= _sections.size()) return NULL;
	return &(_sections[i]);
}

Lane* Road::operator[](const GlobalLaneId& globalId) {
	LaneSection* section = (*this)[globalId.sectionId];
	return section == NULL ? NULL : (*section)[globalId.laneId];
}

LaneSection* Road::front() {
	return _sections.size() > 0 ? &_sections.front() : NULL;
}

Lane* Road::front2() {
	if (_sections.size() == 0) return NULL;
	return _sections[0].front();
}

LaneSection* Road::back() {
	return _sections.size() > 0 ? &_sections.back() : NULL;
}

Lane* Road::next(const Lane* lane) {
	int sectionId = lane->get_globalId().sectionId;
	if (sectionId >= _sections.size()) return NULL;
	Lane* next = _sections[sectionId].next(lane);
	if (next) return next;
	if (++sectionId >= _sections.size()) return NULL;
	return _sections[sectionId].front();
}

LaneSection* Road::next(const LaneSection* section) {
	const Lane* lane = section->front();
	int sectionId = lane->get_globalId().sectionId;
	if (sectionId >= _sections.size() - 1) return NULL;
	return &_sections[sectionId + 1];
}

const LaneSection* Road::operator[](int i) const {
	if (i < 0 || i >= _sections.size()) return NULL;
	return &(_sections[i]);
}

const Lane* Road::operator[](const GlobalLaneId& globalId) const {
	const LaneSection* section = (*this)[globalId.sectionId];
	return section == NULL ? NULL : (*section)[globalId.laneId];
}

const LaneSection* Road::front() const {
	return _sections.size() > 0 ? &_sections.front() : NULL;
}

const Lane* Road::front2() const {
	if (_sections.size() == 0) return NULL;
	return _sections[0].front();
}

const LaneSection* Road::back() const {
	return _sections.size() > 0 ? &_sections.back() : NULL;
}

const Lane* Road::next(const Lane* lane) const {
	int sectionId = lane->get_globalId().sectionId;
	if (sectionId >= _sections.size()) return NULL;
	const Lane* next = _sections[sectionId].next(lane);
	if (next) return next;
	if (++sectionId >= _sections.size()) return NULL;
	return _sections[sectionId].front();
}

const LaneSection* Road::next(const LaneSection* section) const {
	const Lane* lane = section->front();
	int sectionId = lane->get_globalId().sectionId;
	if (sectionId >= _sections.size() - 1) return NULL;
	return &_sections[sectionId + 1];
}

RELATION::RelationType Road::get_relation(const string &id) const {
	for (int i = 0; i < _predecessors.size(); ++i)
		if (id == _predecessors[i].elementId) return RELATION::PREDECESSOR;
	for (int i = 0; i < _successors.size(); ++i)
		if (id == _successors[i].elementId) return RELATION::SUCCESSOR;
	return RELATION::NONE;
}

int Road::get_section_index(double s) const {
	if (_sections.size() == 0) return -1;
	int i = 1, n = _sections.size();
	for ( ; i < n; ++i) {
		if (_sections[i].get_sOffset() > s)
			break;
	}
	return i - 1;
}

const double Road::get_lane_offset(double s) const {
	if (_lane_offsets.size() == 0) return 0.0;
	int i = 0;
	for ( ; i < _lane_offsets.size() - 1; ++i)
		if (_lane_offsets[i + 1].get_s() > s)
			break;
	return _lane_offsets[i].get_offset(s);
}

const double Road::get_elevation(double s) const {
	if (_elevations.size() == 0) return 0.0;
	int i = 0;
	for ( ; i < _elevations.size() - 1; ++i)
		if (_elevations[i + 1].get_s() > s) break;
	return _elevations[i].get_elevation(s);
}

void Road::append_predecessor(const Link &link) {
	for (int i = 0; i < _predecessors.size(); ++i)
		if (_predecessors[i].elementId == link.elementId) return;
	_predecessors.push_back(link);
}

void Road::append_successor(const Link &link) {
	for (int i = 0; i < _successors.size(); ++i)
		if (_successors[i].elementId == link.elementId) return;
	_successors.push_back(link);
}

void Road::init_points() {
	for (int i = 0; i < _sections.size(); ++i) {
		double section_len = (i + 1 == _sections.size() ? _length : _sections[i + 1].get_sOffset()) - _sections[i].get_sOffset();
		_sections[i].set_length(section_len);
	}
	vector<vector<Point> > lanes_points;
	vector<Point> refline_points = _refline.get_refline(0.1);
	int sectionId = 0;
	for (int i = 0; i < refline_points.size() + 1; ++i) {
		double s = i == refline_points.size() ? refline_points[i - 1].s : refline_points[i].s;
		double nexts = sectionId + 1 == _sections.size() ? 1e10 : _sections[sectionId].get_sOffset() + _sections[sectionId].get_length();
		if (i == refline_points.size() || s > nexts) {
			vector<vector<Point> > temp_lanes = cutPoints(lanes_points);
			_refline_points.insert(_refline_points.end(), temp_lanes.back().begin(), temp_lanes.back().end());
			for (int j = 0; j < temp_lanes[0].size(); ++j) {
				vector<Point> temp_points;
				for (int k = 0; k < temp_lanes.size() - 1; ++k) temp_points.push_back(temp_lanes[k][j]);
				_sections[sectionId].append_points(temp_points);
			}
			if (i == refline_points.size()) break;
			lanes_points.clear();
			++sectionId;
		}
		double z0 = get_elevation(s);
		double bias_l = get_lane_offset(s);
		vector<double> ls = _sections[sectionId].get_l_by_s(s);
		for (int k = 0; k < ls.size(); ++k) ls[k] += bias_l;
		
		double x0 = refline_points[i].x;
		double y0 = refline_points[i].y;
		z0 += refline_points[i].z;
		double theta = refline_points[i].theta;
		double pointk = refline_points[i].k;
		if (lanes_points.size() == 0) lanes_points.resize(ls.size() + 1);
		for (int k = 0; k < ls.size(); ++k) {
			double x = x0 - ls[k] * sin(theta);
			double y = y0 + ls[k] * cos(theta);
			lanes_points[k].push_back(Point(x, y, z0, s, ls[k], theta, pointk));
		}
		lanes_points[ls.size()].push_back(refline_points[i]);
		lanes_points[ls.size()].back().z = z0;

	}
	// cout << _id << " ------------" << endl;
	// for (int i = 0; i < _refline_points.size(); ++i)
	// 	_refline_points[i].print();
}

vector<vector<Point> > Road::cutPoints(const vector<vector<Point> >& lanes_points) {
	int m = lanes_points.size();
	int n = lanes_points[0].size();
	vector<vector<Point> > result;
	vector<vector<double> > bounds;
	result.resize(m);
	bounds.resize(m);
	for (int k = 0; k < m; ++k) {
		bounds[k].resize(4);
		bounds[k][0] = -2 * M_PI; bounds[k][1] = 2 * M_PI; bounds[k][2] = -M_PI / 2; bounds[k][3] = M_PI / 2;
	}

	int s = 0;
	for (int k = 0; k < m; ++k) result[k].push_back(lanes_points[k][0]);
	for (int i = 1; i < n; ++i) {
		bool flag = true;
		for (int k = 0; k < m; ++k)
			if (!update_bounds(lanes_points[k][s], lanes_points[k][i], bounds[k])) flag = false;
		if (!flag) {
			for (int k = 0; k < m; ++k) {
				result[k].push_back(lanes_points[k][i - 1]);
				bounds[k][0] = -2 * M_PI; bounds[k][1] = 2 * M_PI; bounds[k][2] = -M_PI / 2; bounds[k][3] = M_PI / 2;
				update_bounds(lanes_points[k][i - 1], lanes_points[k][i], bounds[k]);
			}
			s = i - 1;
		}
	}
	for (int k = 0; k < m; ++k) result[k].push_back(lanes_points[k][n - 1]);
	return result;
}

bool Road::update_bounds(const Point& p1, const Point& p2, vector<double>& bounds) {
	double max_length = Max_Point_Distance;
	double dist = p1._distance(p2);
	if (dist > max_length) return false;
	if (dist / XY_Precision < 2.0) return true;
	Point p12 = p2 - p1;
	double alpha = p12.yaw();
	double beta = asin(p12.z / dist);
	double rangeXY = XY_Precision / dist;
	double rangeZ = Z_Precision / dist;

	double lb = bounds[0], hb = bounds[1];
	double alb = alpha - rangeXY;
	double ahb = alpha + rangeXY;
	if (!(alb > hb || ahb < lb)) {
		if (ahb < hb) hb = ahb;
		if (alb > lb) lb = alb;
	}
	else {
		if (alpha > M_PI / 2) {
			alb -= 2 * M_PI;
			ahb -= 2 * M_PI;
		}
		else {
			alb += 2 * M_PI;
			ahb += 2 * M_PI;
		}
		if (!(alb > hb || ahb < lb)) {
			if (ahb < hb) hb = ahb;
			if (alb > lb) lb = alb;
		}
		else
			return false;
	}
	bounds[0] = lb;
	bounds[1] = hb;

	lb = bounds[2];
	hb = bounds[3];
	alb = beta - rangeZ;
	ahb = beta + rangeZ;
	if (!(alb > hb || ahb < lb)) {
		if (ahb < hb) hb = ahb;
		if (alb > lb) lb = alb;
	}
	else
		return false;
	bounds[2] = lb;
	bounds[3] = hb;
	return true;
}

}