/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: lane_converter.cpp
*   Author  : lubing.han
*   Date    : 2017-03-07
*   Describe:
*
********************************************************/
#include "lane_converter.h"
#include <algorithm>
namespace opendrive {

MapInfo::MapInfo(int envId, const GlobalLaneId &laneId, bool re, double es, double ms, double l):
	globalEnvId(envId), globalLaneId(laneId), reversed(re), envSOffset(es), mapSOffset(ms), length(l) {
}

double MapInfo::map2envS(double s) const {
	s -= mapSOffset;
	// if (s < -1e-3 || s > length + 1e-3) return -1.0;
	if (reversed) s = length - s;
	return envSOffset + s;
}

double MapInfo::env2mapS(double s) const {
	s -= envSOffset;
	// if (s < -1e-3 || s > length + 1e-3) return -1.0;
	if (reversed) s = length - s;
	return mapSOffset + s;
}

void MapInfo::get_envSLT(Point &p) const {
	p.s = map2envS(p.s);
	if (reversed) {
		p.l = -p.l;
		p.theta += M_PI;
		if (p.theta > M_PI) p.theta -= 2 * M_PI;
	}
}

void MapInfo::get_mapSLT(Point &p) const {
	p.s = env2mapS(p.s);
	if (reversed) {
		p.l = -p.l;
		p.theta += M_PI;
		if (p.theta > M_PI) p.theta -= 2 * M_PI;
	}
}


EnvLane::EnvLane(const EnvLane& envLane, double startS, double endS) {
	const vector<GlobalLaneId>& globalIds = envLane.get_globalIds();
	const vector<MapInfo>& mapInfos = envLane.get_mapInfos();
	int i = 0;
	for (; i < globalIds.size(); ++i)
		if (mapInfos[i].envSOffset + mapInfos[i].length > startS) break;
	if (i == globalIds.size()) _envSOffset = mapInfos[i - 1].envSOffset + mapInfos[i - 1].length;
	else _envSOffset = mapInfos[i].envSOffset;
	_length = 0.0;
	for (; i < globalIds.size(); ++i) {
		if (mapInfos[i].envSOffset > endS) break;
		_globalIds.push_back(globalIds[i]);
		_mapInfos.push_back(mapInfos[i]);
		_length += mapInfos[i].length;
	}

	_globalEnvId = envLane._globalEnvId;
	_tags = envLane._tags;

	for (int i = 0; i < envLane._stopLines.size(); ++i)
		if (envLane._stopLines[i].in_range(_envSOffset, _envSOffset + _length)) _stopLines.push_back(envLane._stopLines[i]);
	for (int i = 0; i < envLane._crossWalks.size(); ++i)
		if (envLane._crossWalks[i].in_range(_envSOffset, _envSOffset + _length)) _crossWalks.push_back(envLane._crossWalks[i]);
	for (int i = 0; i < envLane._parkingSpaces.size(); ++i)
		if (envLane._parkingSpaces[i].in_range(_envSOffset, _envSOffset + _length)) _parkingSpaces.push_back(envLane._parkingSpaces[i]);
	for (int i = 0; i < envLane._normalObjects.size(); ++i)
		if (envLane._normalObjects[i].in_range(_envSOffset, _envSOffset + _length)) _normalObjects.push_back(envLane._normalObjects[i]);
	for (int i = 0; i < envLane._junctionSs.size(); ++i)
		if (envLane._junctionSs[i].second > _envSOffset && envLane._junctionSs[i].first < _envSOffset + _length)
			_junctionSs.push_back(envLane._junctionSs[i]);
	for (int i = 0; i < envLane._innerLaneChanges.size(); ++i)
		if (envLane._innerLaneChanges[i].first + 1e-3 > _envSOffset && envLane._innerLaneChanges[i].first + 1e-3 < _envSOffset + _length)
			_innerLaneChanges.push_back(envLane._innerLaneChanges[i]);
	for (int i = 0; i < envLane._outerLaneChanges.size(); ++i)
		if (envLane._outerLaneChanges[i].first + 1e-3 > _envSOffset && envLane._outerLaneChanges[i].first + 1e-3 < _envSOffset + _length)
			_outerLaneChanges.push_back(envLane._outerLaneChanges[i]);
}

bool EnvLane::get_nearest_stopLine(double s, pair<double, string>& stopLine) const {
	double ns = -1;
	for (int i = 0; i < _stopLines.size(); ++i) {
		if (_stopLines[i].get_s() > s && (ns < 0 || _stopLines[i].get_s() < ns)) {
			ns = _stopLines[i].get_s();
			stopLine.first = ns, stopLine.second = _stopLines[i].get_signalId();
		}
	}
	return ns < 0 ? false : true;
}

bool EnvLane::get_nearest_crossWalk(double s, pair<double, double>& ns) const {
	ns.first = -1;
	for (int i = 0; i < _crossWalks.size(); ++i) {
		if (_crossWalks[i].get_maxS() > s && (ns.first < 0 || _crossWalks[i].get_maxS() < ns.second))
			ns.first = _crossWalks[i].get_minS(), ns.second = _crossWalks[i].get_maxS();
	}
	return ns.first < 0 ? false : true;
}

bool EnvLane::get_nearest_junctionS(double s, pair<double, double>& ns) const {
	ns.first = -1;
	for (int i = 0; i < _junctionSs.size(); ++i) {
		if (_junctionSs[i].second > s && (ns.first < 0 || _junctionSs[i].first < ns.first))
			ns = _junctionSs[i];
	}
	return ns.first < 0 ? false : true;
}

bool EnvLane::check_laneChange(int direction, double startS, double endS) {
	if (endS < 0) endS = startS;
	if (direction == 0 || startS < _envSOffset || endS > _envSOffset + _length) return false;
	const vector<pair<double, bool> >* pLaneChange;
	if (direction < 0) pLaneChange = &_innerLaneChanges;
	else pLaneChange = &_outerLaneChanges;
	int si = 1, ei;
	for ( ; si < pLaneChange->size(); ++si)
		if ((*pLaneChange)[si].first >= startS) break;
	--si;
	for (ei = si + 1; ei < pLaneChange->size(); ++ei)
		if ((*pLaneChange)[ei].first >= startS) break;
	--ei;
	for (int i = si; i <= ei; ++i)
		if (!(*pLaneChange)[i].second) return false;
	return true;
}



bool EnvLane::get_parkingSpace(string id, ParkingSpace& parkingSpace) const {
	for (int i = 0; i < _parkingSpaces.size(); ++i) {
		if (id == _parkingSpaces[i].get_id()) {
			parkingSpace = _parkingSpaces[i];
			return true;
		}
	}
	return false;
}

bool EnvLane::get_parking_s(string id, double& s) const {
	for (int i = 0; i < _parkingSpaces.size(); ++i) {
		if (id == _parkingSpaces[i].get_id()) {
			s = _parkingSpaces[i].get_s();
			return true;
		}
	}
	return false;
}

double EnvLane::get_nearest_junctionS(double curS) const {
	int i = get_index_by_s(curS), j;
	if (i < 0) return -1;
	const RoadMap* roadmap = RoadMap::get_instance();
	for (j = i; j < _mapInfos.size(); ++j)
		if ((*roadmap)[_globalIds[j].id]->get_junction() != "-1") break;
	if (j == _mapInfos.size()) return -1;
	if (j == i) return curS;
	return _mapInfos[j].envSOffset;
}

const GlobalLaneId* EnvLane::get_globalId(double s) const {
	int index = get_index_by_s(s);
	if (index < 0) return NULL;
	else return &(_globalIds[index]);
}

const MapInfo* EnvLane::get_mapInfo(double s) const {
	int index = get_index_by_s(s);
	if (index < 0) return NULL;
	else return &(_mapInfos[index]);
}

void EnvLane::append_lane(const Lane* lane, bool re, string direction) {
	double es;
	if (direction == "backward") {
		es = _mapInfos.size() == 0 ? _envSOffset : _envSOffset - lane->get_length();
		_envSOffset = es;
	}
	else 
		es = _mapInfos.size() == 0 ? _envSOffset : _mapInfos.back().envSOffset + _mapInfos.back().length;
	_length += lane->get_length();
	double ms = lane->get_sOffset();
	double l = lane->get_length();
	if (direction == "backward") {
		_mapInfos.insert(_mapInfos.begin(), MapInfo(_globalEnvId, lane->get_globalId(), re, es, ms, l));
		_globalIds.insert(_globalIds.begin(), lane->get_globalId());
	}
	else {
		_mapInfos.push_back(MapInfo(_globalEnvId, lane->get_globalId(), re, es, ms, l));
		_globalIds.push_back(lane->get_globalId());
	}
}

int EnvLane::get_index_by_s(double s) const {
	if (s + 0.01 < _mapInfos.front().envSOffset || s - 0.01 > _mapInfos.back().envSOffset + _mapInfos.back().length)
		return -1;
	int lo = 0, hi = _mapInfos.size();
	while (lo + 1 < hi) {
		int md = (lo + hi) / 2;
		if (_mapInfos[md].envSOffset <= s) lo = md;
		else hi = md;
	}
	return lo;
}

void EnvLane::print() const {
	cout << "EnvLane: " << _globalEnvId << ", type: " << _type << ", s: " << _envSOffset << ". len: " << _length << endl;
	cout << "  Tags: ";
	for (int i = 0; i < _tags.size(); ++i)
		cout << "(" << _tags[i].first << ", " << _tags[i].second << ") ";
	cout << endl;

	cout << "  StopLines: ";
	for (int i = 0; i < _stopLines.size(); ++i)
		cout << "(" << _stopLines[i].get_s() << ", " << _stopLines[i].get_signalId() << ") ";
	cout << endl;

	cout << "  CrossWalks: ";
	for (int i = 0; i < _crossWalks.size(); ++i)
		cout << "(" << _crossWalks[i].get_minS() << ", " << _crossWalks[i].get_maxS() << ") ";
	cout << endl;

	cout << "  Junctions: ";
	for (int i = 0; i < _junctionSs.size(); ++i)
		cout << "(" << _junctionSs[i].first << ", " << _junctionSs[i].second << ") ";
	cout << endl;

	cout << "  InnerLaneChanges: ";
	for (int i = 0; i < _innerLaneChanges.size(); ++i)
		cout << "(" << _innerLaneChanges[i].first << ", " << _innerLaneChanges[i].second << ") ";
	cout << endl;

	cout << "  OuterLaneChanges: ";
	for (int i = 0; i < _outerLaneChanges.size(); ++i)
		cout << "(" << _outerLaneChanges[i].first << ", " << _outerLaneChanges[i].second << ") ";
	cout << endl;

	cout << "  Lanes: ";
	for (int i = 0; i < _globalIds.size(); ++i)
		cout << "(" << _globalIds[i].id << ", " << _globalIds[i].sectionId << ", " << _globalIds[i].laneId << ") ";
	cout << endl;
}

bool EnvLane::is_Tlane() const {
	for (int i = 0; i < _tags.size(); i++) {
		if (_tags[i].first == 'T') {
			return true;
		}
	}
	return false;
}

bool EnvLane::is_Rlane() const {
	for (int i = 0; i < _tags.size(); i++) {
		if (_tags[i].first == 'R'){
			return true;
		}
	}
	return false;
}

bool EnvLane::is_Tlane(int & id) const {
	for (int i = 0; i < _tags.size(); i++) {
		if (_tags[i].first == 'T') {
			id = _tags[i].second;
			return true;
		}
	}
	return false;
}

bool EnvLane::is_Rlane(int & id) const {
	for (int i = 0; i < _tags.size(); i++) {
		if (_tags[i].first == 'R'){
			id = _tags[i].second;
			return true;
		}
	}
	return false;
}

bool EnvLane::is_Flane() const {
	for (int i = 0; i < _tags.size(); i++) {
		if (_tags[i].first == 'F') return true;
	}
	return false;
}

bool EnvLane::is_Jlane() const {
	for (int i = 0; i < _tags.size(); i++) {
		if (_tags[i].first == 'J') return true;
	}
	return false;
}

bool EnvLane::get_tagindex(char type, int& index) const {
	for (int i = 0; i < _tags.size(); i++) {
		if (_tags[i].first == type) {
			index =  _tags[i].second;
			return true;
		}
	}
	return false;
}

vector<Point> EnvLane::get_center_line() const {
	vector<Point> points;
	const RoadMap* roadmap = RoadMap::get_instance();
	for (int i = 0; i < _globalIds.size(); ++i) {
		vector<Point> temp_points = (*roadmap)[_globalIds[i]]->get_center();
		for (int j = 0; j < temp_points.size(); ++j)
			_mapInfos[i].get_envSLT(temp_points[j]);
		if (_mapInfos[i].reversed) reverse(temp_points.begin(), temp_points.end());
		points.insert(points.end(), temp_points.begin(), temp_points.end());
	}
	return points;
}

void EnvLane::get_border_lines(vector<Point> &points1, vector<Point> &points2) const {
	points1.clear(), points2.clear();
	const RoadMap* roadmap = RoadMap::get_instance();
	for (int i = 0; i < _globalIds.size(); ++i) {
		vector<Point> temp_points1 = (*roadmap)[_globalIds[i]]->get_inner_points();
		vector<Point> temp_points2 = (*roadmap)[_globalIds[i]]->get_lane_points();
		for (int j = 0; j < temp_points1.size() && j < temp_points2.size(); ++j) {
			_mapInfos[i].get_envSLT(temp_points1[j]);
			_mapInfos[i].get_envSLT(temp_points2[j]);
		}
		if (_mapInfos[i].reversed) {
			reverse(temp_points1.begin(), temp_points1.end());
			reverse(temp_points2.begin(), temp_points2.end());
		}
		points1.insert(points1.end(), temp_points1.begin(), temp_points1.end());
		points2.insert(points2.end(), temp_points2.begin(), temp_points2.end());
	}
}

}
