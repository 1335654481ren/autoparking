/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: road_fun.cpp
*   Author  : lubing.han
*   Date    : 2017-03-21
*   Describe:
*
********************************************************/
#include "road_fun.h"
#include <cmath>
using namespace std;

namespace opendrive {

const Lane* sl2lane(const Road* road, double s, double l) {
	int i = road->get_section_index(s);
	if (i < 0) return NULL;
	const LaneSection* section = (*road)[i];
	vector<double> ls = section->get_l_by_s(s);
	double bias_l = road->get_lane_offset(s);

	int j = 1;
	ls[0] += bias_l;
	for ( ; j < ls.size(); ++ j) {
		ls[j] += bias_l;
		if (l >= ls[j] && l <= ls[j - 1]) break;
	}
	if (j == ls.size()) return NULL;
	int lane_id = section->get_lane_id(j - 1);
	const Lane* lane = (*section)[lane_id];
	if (lane_id <= 0) lane = section->next(lane);
	return lane;
}

bool sl2xyz(const Road* road, Point& p) {
	int i = road->get_section_index(p.s);
	if (i < 0) return false;
	p = sl2point((*road)[i]->front(), p.s, p.l);
	if (sl2lane(road, p.s, p.l)) return true;
	else return false;
}

vector<Marker> get_markers(const Road* road, const GlobalLaneId &laneId, const MarkerFilter &filter) {
	vector<Marker> rmarkers;
	const vector<Marker> markers = road->get_markers();
	for (int i = 0; i < markers.size(); ++i) {
		if (filter.type != "" && markers[i].get_type() != filter.type) continue;
		double s = markers[i].get_s();
		// laneId.print();
		// cout << "A "<< markers[i].get_type() << " " << markers[i].get_s() << endl;
		if (road->get_section_index(s) != laneId.sectionId) continue;
		// cout << "B "<< markers[i].get_type() << " " << markers[i].get_s() << endl;
		if (markers[i].get_type() == "stopLine" && !markers[i].is_valid(laneId.laneId)) continue;
		rmarkers.push_back(markers[i]);
	}
	return rmarkers;
}

GlobalLaneId get_nearest_lane(const Road* road, double s, const geometry_msgs::Point& p) {
	Point op(p.x, p.y, p.z);
	const LaneSection* section = (*road)[road->get_section_index(s)];
	GlobalLaneId laneId("", 0, 0);
	double min_dist = 1e10;
	for (const Lane* lane = section->front(); lane; lane = section->next(lane)) {
		if (lane->get_id() == 0) continue;
		Point temp = s2center_point(lane, s);
		if (op.dist2(temp) < min_dist) {
			min_dist = op.dist2(temp);
			laneId = lane->get_globalId();
		}
	}
	return laneId;
}

}