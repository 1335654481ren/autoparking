/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: junction_env.cpp
*   Author  : lubing.han
*   Date    : 2017-03-22
*   Describe:
*
********************************************************/

#include "junction_env.h"
#include "envlane_fun.h"
#include "map_api_parameters.h"
using namespace std;

namespace opendrive {

void JunctionEnv::reset_env(int& globalEnvId, string junctionId,
	const vector<GlobalLaneId>& fromLanes, const vector<GlobalLaneId>& viaLanes, const vector<GlobalLaneId>& toLanes,
	const vector<int> fromGroups, const vector<int> toGroups) {
	_junctionId = junctionId;
	_fromLanes = fromLanes;
	_viaLanes = viaLanes;
	_toLanes = toLanes;
	_fromGroups = fromGroups;
	_toGroups = toGroups;
	_roadMap = RoadMap::get_instance();

	for (int i = 0; i < fromLanes.size(); ++i) {
		EnvLane envLane(globalEnvId, 0.0);
		
		const Lane* lane = (*_roadMap)[fromLanes[i]];
		if (lane->get_id() * Drive_Direction < 0) envLane.append_lane(lane, false, "forward");
		else envLane.append_lane(lane, true, "forward");

		double sumS = lane->get_length();
		while (sumS < Junction_Extend_Length) {
			vector<GlobalLaneId> globalIds;
			if (lane->get_id() * Drive_Direction < 0) globalIds = lane->get_predecessors();
			else if (lane->get_id() * Drive_Direction > 0) globalIds = lane->get_successors();
			else break;
			if (globalIds.size() != 1) break;
			lane = (*_roadMap)[globalIds[0]];
			if (lane->get_id() * Drive_Direction < 0) {
				envLane.append_lane(lane, false, "backward");
				sumS += lane->get_length();
			}
			else if (lane->get_id() * Drive_Direction > 0) {
				envLane.append_lane(lane, true, "backward");
				sumS += lane->get_length();
			}
			else break;
		}


		lane = (*_roadMap)[viaLanes[i]];
		if (lane->get_id() * Drive_Direction < 0) envLane.append_lane(lane, false, "forward");
		else envLane.append_lane(lane, true, "forward");
		double junctionStartS = envLane.get_mapInfos().back().envSOffset;
		double junctionEndS = junctionStartS + envLane.get_mapInfos().back().length;
		envLane.set_junctionStartS(junctionStartS);
		envLane.set_junctionEndS(junctionEndS);
		envLane.append_junctionS(make_pair(junctionStartS, junctionEndS));
		lane = (*_roadMap)[toLanes[i]];
		if (lane->get_id() * Drive_Direction < 0) envLane.append_lane(lane, false, "forward");
		else envLane.append_lane(lane, true, "forward");

		sumS = lane->get_length();
		while (sumS < Junction_Extend_Length) {
			vector<GlobalLaneId> globalIds;
			if (lane->get_id() * Drive_Direction < 0) globalIds = lane->get_successors();
			else if (lane->get_id() * Drive_Direction > 0) globalIds = lane->get_predecessors();
			else break;
			if (globalIds.size() != 1) break;
			lane = (*_roadMap)[globalIds[0]];
			if (lane->get_id() * Drive_Direction < 0) {
				envLane.append_lane(lane, false, "forward");
				sumS += lane->get_length();
			}
			else if (lane->get_id() * Drive_Direction > 0) {
				envLane.append_lane(lane, true, "forward");
				sumS += lane->get_length();
			}
			else break;
		}
		envLane.append_tag(make_pair('J', _fromGroups[i] * 4 + _toGroups[i]));
		_junctionLanes[globalEnvId++] = envLane;
	}
	generate_conflictS();
}

void JunctionEnv::print() const {
	for (int i = 0; i < _fromLanes.size(); ++i) {
		cout << "from (" << _fromLanes[i].id << ", " << _fromLanes[i].laneId << ") ";
		cout << "via (" << _viaLanes[i].id << ", " << _viaLanes[i].laneId << ") ";
		cout << "to (" << _toLanes[i].id << ", " << _toLanes[i].laneId << ") ";
		cout << "group: (" << _fromGroups[i] << ", " << _toGroups[i] << ")" << endl;
	}
}

const EnvLane* JunctionEnv::get_envLane(GlobalLaneId inLane, GlobalLaneId outLane) const {
	map<int, EnvLane>::const_iterator iter = _junctionLanes.begin();
	for (int i = 0; i < _fromLanes.size(); ++i, ++iter) {
		if (_fromLanes[i] == inLane && _toLanes[i] == outLane)
			return &(iter->second);
	}
	return NULL;
}

int JunctionEnv::get_mode(GlobalLaneId inLane, GlobalLaneId outLane) const {
	for (int i = 0; i < _fromLanes.size(); ++i) {
		if (_fromLanes[i] == inLane && _toLanes[i] == outLane)
			return 4 * _fromGroups[i] + _toGroups[i];
	}
	ROS_ERROR("JunctionEnv error: can not get mode");
	cout << "Junction ID: " << _junctionId << endl;
	inLane.print();
	outLane.print();
	return -1;
}

void JunctionEnv::get_conflictS(GlobalLaneId inLane, GlobalLaneId outLane,
	vector<int>& envIds, vector<double>& selfS, vector<double>& otherS) const {
	int i = 0;
	for ( ; i < _fromLanes.size(); ++i) if (_fromLanes[i] == inLane && _toLanes[i] == outLane) break;
	if (i == _fromLanes.size()) return;
	map<int, EnvLane>::const_iterator iter = _junctionLanes.begin();
	for (int j = 0; j < _fromLanes.size(); ++j, ++iter) {
		if (i == j) continue;
		if (_conflictS[i][j] > 0) {
			envIds.push_back(iter->first);
			selfS.push_back(_conflictS[i][j]);
			otherS.push_back(_conflictS[j][i]);
		}
	}
}

void JunctionEnv::generate_conflictS() {
	_conflictS.clear();
	for (map<int, EnvLane>::iterator i = _junctionLanes.begin(); i != _junctionLanes.end(); ++i) {
		_conflictS.push_back(vector<double>());
		vector<Point> psi;
		get_center_line(_roadMap, &(i->second), psi);
		for (map<int, EnvLane>::iterator j = _junctionLanes.begin(); j != _junctionLanes.end(); ++j) {
			vector<Point> psj;
			get_center_line(_roadMap, &(j->second), psj);
			double minS = -1.0;
			double minD = 1e10;
			for (int k = 0; k + 1 < psi.size(); ++k) {
				if (psi[k].s < i->second.get_junctionStartS() || psi[k + 1].s > i->second.get_junctionEndS()) continue;
				for (int l = 0; l + 1 < psj.size(); ++l) {
					if (psj[l].s < j->second.get_junctionStartS() || psj[l + 1].s > j->second.get_junctionEndS()) continue;
					double t1, t2;
					double d = get_min_dist(psi[k], psi[k + 1], psj[l], psj[l + 1], t1, t2);
					if (d < 2.0 && d < minD) {
						minD = d;
						minS = (1 - t1) * psi[k].s + t1 * psi[k + 1].s;
					}
				}
			}
			_conflictS.back().push_back(minS);
		}
	}
}

double JunctionEnv::get_min_dist(Point p1, Point p2, Point q1, Point q2, double &t1, double &t2) {
	p1.z = p2.z = q1.z = q2.z = 0.0;
	if (p1.dist2(p2) < 1e-10 || q1.dist2(q2) < 1e-10) return 1e10;
	double pa = p2.x - p1.x, pb = p2.y - p1.y;
	double qa = q2.x - q1.x, qb = q2.y - q1.y;
	double tp, tq;
	// pa * tp + p1.x == qa * tq + q1.x;
	// pb * tp + p1.y == qb * tq + q1.y;
	// pb * (qa * tq + q1.x - p1.x) == pa * (qb * tq + q1.y - p1.y);
	// (pb * qa - pa * qb) * tq = pa * (q1.y - p1.y) - pb * (q1.x - p1.x);
	if (fabs(pb * qa - pa * qb) > 1e-10) {
		tq = (pa * (q1.y - p1.y) - pb * (q1.x - p1.x)) / (pb * qa - pa * qb);
		if (fabs(pa) > 1e-10)
			tp = (qa * tq + q1.x - p1.x) / pa;
		else if (fabs(pb) > 1e-10)
			tp = (qb * tq + q1.y - p1.y) / pb;
		else
			tp = -1;
		if (0 <= tp && tp <= 1 && 0 <= tq && tq <= 1) {
			t1 = tp, t2 = tq;
			return 0.0;
		}
	}

	double d1q, t1q; d1q = p1.dist2(q1, q2, t1q);
	double d2q, t2q; d2q = p2.dist2(q1, q2, t2q);
	double d1p, t1p; d1p = q1.dist2(p1, p2, t1p);
	double d2p, t2p; d2p = q2.dist2(p1, p2, t2p);
	double mind = d1q; mind = fmin(mind, d2q); mind = fmin(mind, d1p); mind = fmin(mind, d2p);
	if (mind == d1q) t1 = 0, t2 = t1q;
	else if (mind == d2q) t1 = 1, t2 = t2q;
	else if (mind == d1p) t1 = t1p, t2 = 0;
	else t1 = t2p, t2 = 1;
	return sqrt(mind);
}

}
