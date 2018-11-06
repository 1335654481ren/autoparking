/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: section_fun.cpp
*   Author  : lubing.han
*   Date    : 2017-03-21
*   Describe:
*
********************************************************/
#include "section_fun.h"
#include <algorithm>
using namespace std;

namespace opendrive {

vector<pair<double, bool> > get_laneChanges(const LaneSection* section, int laneId, bool outer, bool reversed) {
	vector<pair<double, bool> > laneChanges;
	const Lane *innerLane = NULL, *outerLane = NULL;
	if ((2 * reversed - 1) * Drive_Direction * laneId > 0) {
		innerLane = laneId > 0 ? section->get_decrease_lane(laneId) : section->get_increase_lane(laneId);
		outerLane = (*section)[laneId];
	}
	else if ((2 * reversed - 1) * Drive_Direction * laneId < 0) {
		innerLane = (*section)[laneId];
		outerLane = laneId > 0 ? section->get_decrease_lane(laneId) : section->get_increase_lane(laneId);
	}
	const Lane* lane = outer ? outerLane : innerLane;
	if (!lane) return laneChanges;
	int innerId = innerLane->get_id(), outerId = outerLane->get_id();
	const vector<RoadMark>& roadMarks = lane->get_roadmarks();
	if (roadMarks.size() == 0)
		laneChanges.push_back(make_pair(lane->get_sOffset(), false));
	else {
		for (int i = 0; i < roadMarks.size(); ++i) {
			double s = roadMarks[i].get_sOffset() + lane->get_sOffset();
			string targetDirection;
			if ((innerId < outerId && outer) || (innerId > outerId && !outer)) targetDirection = "increase";
			else if ((innerId < outerId && !outer) || (innerId > outerId && outer)) targetDirection = "decrease";
			bool flag = roadMarks[i].checkLaneChange(targetDirection);
			laneChanges.push_back(make_pair(s, flag));
		}
	}
	if (reversed) {
		for (int i = 0; i + 1 < laneChanges.size(); ++i)
			laneChanges[i].first = laneChanges[i + 1].second;
		laneChanges.back().first = lane->get_sOffset() + lane->get_length();
		reverse(laneChanges.begin(), laneChanges.end());
	}
	return laneChanges;
}

}