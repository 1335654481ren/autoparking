/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: lane_node.cpp
*   Author  : lubing.han
*   Date    : 2017-03-21
*   Describe:
*
********************************************************/
#include "lane_node.h"
using namespace std;

namespace opendrive {

LaneNode::LaneNode() {
	_visited = false;
	_lChild = _rChild = NULL;
}

LaneNode::~LaneNode() {
	clear();
}

double LaneNode::get_minDist() const {
 return _minDist;
}

double LaneNode::get_maxDist() const {
 return _maxDist;
}

string LaneNode::get_id() const {
	return _id;
}

void LaneNode::set_id(const string id) {
	_id = id;
}

void LaneNode::set_points(const vector<Point>& points) {
	set_points(points, 0, (int)points.size() - 1);
}

void LaneNode::set_points(const vector<Point>& points, int s, int e) {
	clear();
	_range = 0.0;
	_p1 = points[s];
	_p2 = points[e];
	while (_p1.theta - _p2.theta > M_PI) _p1.theta -= 2 * M_PI;
	while (_p2.theta - _p1.theta > M_PI) _p2.theta -= 2 * M_PI;
	if (s + 1 == e) return;
	for (int i = s + 1; i < e; ++i) {
		double d2, t;
		d2 = points[i].dist2xy(_p1, _p2, t);
		if (d2 > _range) _range = d2;
	}
	_range = sqrt(_range);
	int mi = (s + e) / 2;
	_lChild = new LaneNode();
	_lChild->set_points(points, s, mi);
	_rChild = new LaneNode();
	_rChild->set_points(points, mi, e);
}

void LaneNode::reset() {
	if (_visited) {
		_visited = false;
		if (_lChild) _lChild->reset();
		if (_rChild) _rChild->reset();
	}
}

void LaneNode::update_once(const Point &p, double& minDist, double& maxDist, double nowDist) {
	if (!_visited) {
		_visited = true;
		double d, t;
		d = sqrt(p.dist2xy(_p1, _p2, t));
		minDist = _minDist = d > _range ? d - _range : 0.0;
		maxDist = _maxDist = d + _range;
		// cout << "Update " << _id << ", " << _range << ", " << _minDist << ", " << _maxDist << endl;
		return;
	}
	minDist = _minDist;
	maxDist = _maxDist;
	if (minDist > nowDist) return;
	if (_lChild == NULL) return;
	double minl, maxl, minr, maxr;
	// minl = maxl = minr = maxr = 1e10;
	// if (!_lChild->isVisited() || _lChild->get_minDist() < _rChild->get_maxDist())
	_lChild->update_once(p, minl, maxl, maxDist < nowDist ? maxDist : nowDist);
	// if (!_rChild->isVisited() || _rChild->get_minDist() < _lChild->get_minDist())
	_rChild->update_once(p, minr, maxr, maxDist < nowDist ? maxDist : nowDist);
	double minlr = minl < minr ? minl : minr;
	double maxlr = maxl < maxr ? maxl : maxr;
	if (minlr > _minDist) minDist = _minDist = minlr;
	if (maxlr < _maxDist) maxDist = _maxDist = maxlr;
}

void LaneNode::get_sl(Point &p) {
	if (_lChild == NULL || !_lChild->isVisited()) {
		double d, t;
		d = sqrt(p.dist2xy(_p1, _p2, t));
		Point p0 = (1 - t) * _p1 + t * _p2;
		// cout << _p1.s << " " << _p2.s << " " << t << endl;
		p.s = p0.s;
		p.theta = p0.theta;
		double flag = -sin(p0.theta) * (p.x - p0.x) + cos(p0.theta) * (p.y - p0.y);
		p.l = p0.l + (flag > 0 ? d : -d);
	}
	else {
		if (_lChild->get_maxDist() < _rChild->get_maxDist())
			_lChild->get_sl(p);
		else
			_rChild->get_sl(p);
	}
}

bool LaneNode::isVisited() const {
	return _visited;
}

bool LaneNode::operator<(const LaneNode& other) const {
	return _minDist < other.get_minDist();
}

void LaneNode::clear() {
	_visited = false;
	if (_lChild) delete _lChild;
	if (_rChild) delete _rChild;
	_lChild = _rChild = NULL;
}

}