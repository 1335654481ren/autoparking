/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: map_environment.cpp
*   Author  : lubing.han
*   Date    : 2017-03-22
*   Describe:
*
********************************************************/

#include "map_environment.h"
#include "roadmap_fun.h"
#include <algorithm>
using namespace std;

namespace opendrive {

MapEnvironment::MapEnvironment(int selfId) {
	_selfId = selfId;
	_parkingId = "null";
	_line_end = false;
	_roadmap = RoadMap::get_instance();
	_trans = CoordinatesTrans::getInstance();
	_grid.data.resize(GRID_WIDTH * GRID_HEIGHT);
	_grid.info.width = GRID_WIDTH;
	_grid.info.height = GRID_HEIGHT;
	_grid.info.resolution = GRID_RESOLUTION;
}

const EnvLane* MapEnvironment::operator[](int id) const {
	map<int, EnvLane>::const_iterator i = _envLanes.find(id);
	return i == _envLanes.end() ? NULL : &(i->second);
}

const vector<EnvLane> MapEnvironment::operator[](const std::pair<char, int> tag) const {
    vector<EnvLane> res;
    for (map<int, EnvLane>::const_iterator i = _envLanes.begin(); i != _envLanes.end(); i++) {
        int index = -1e6;
        if (i->second.get_tagindex(tag.first, index) && index == tag.second) {
            res.push_back(i->second);
        }
    }
    return res;
}

bool MapEnvironment::set_route(Point p) {
	geometry_msgs::Point point;
	point.x = p.x, point.y = p.y, point.z = p.z;

	vector<GlobalLaneId> laneIds;
	const Lane* lane = _roadmap->front3();
	for ( ; lane != NULL; lane = _roadmap->next(lane))
		laneIds.push_back(lane->get_globalId());

	GlobalLaneId result;
	_trans->xyz2sli(-100, p, laneIds, result);

	vector<ParkingSpace> parkings;
	vector<string> ids;
	point2parkingSpaces(_roadmap, point, parkings, ids);

	vector<string> route;
	if (result.id != "") route.push_back(result.id);
	else if (ids.size() > 0) route.push_back(ids[0]);
	else return false;
	_route = route;
	return set_route(route);
}

bool MapEnvironment::set_route(const vector<string>& route) {
	_globalEnvLanes.clear();
	_globalMapLanes.clear();
	_envLanes.clear();
	_mapLanes.clear();
	_route.clear();
	_reversed.clear();
	_routeS.clear();
	_route_lanes.clear();
	_local_lanes.clear();
	_line_end = false;
	fill(_grid.data.begin(), _grid.data.end(), 100);
	_mapState.curS = 0.0;
        _mapState.outOfRoute = true;
	int n = route.size();
	if (n < 1) {
		ROS_ERROR("Invalid route, at least one routes!");
		return false;
	}
	for (int i = 0; i < n; ++i) {
		if (!(*_roadmap)[route[i]]) {
			ROS_ERROR("Invalid route, can not find road %s !", route[i].c_str());
			return false;
		}
		if (i > 0 && (*_roadmap)[route[i]]->get_relation(route[i - 1]) == RELATION::NONE) {
			ROS_ERROR("Invalid route, roads not connected");
			return false;
		}
		if (i < n - 1 && (*_roadmap)[route[i]]->get_relation(route[i + 1]) == RELATION::NONE) {
			ROS_ERROR("Invalid route, roads not connected");
			return false;	
		}
	}
	_route = route;
	for (int i = 0; i < route.size(); ++i) {
		const Road* road = (*_roadmap)[route[i]];
		for (const Lane* lane = road->front2(); lane != NULL; lane = road->next(lane))
			// if (lane->get_type() == "driving") _route_lanes.push_back(lane->get_globalId());
			_route_lanes.push_back(lane->get_globalId());
        }
        get_global_envlanes(_roadmap, _route, _reversed, _routeS, _globalEnvLanes, _globalMapLanes);
	// print(_globalEnvLanes);
        if (_selfId < 0) get_global_junctions(_roadmap, _globalEnvLanes, _junctionEnvs);
	// for (map<string, JunctionEnv>::iterator i = _junctionEnvs.begin() ; i != _junctionEnvs.end(); ++i) {
	// 	print(i->second.get_junctionLanes());
	// 	i->second.print();
	// }
	return true;
}

bool MapEnvironment::set_parkingSpace(geometry_msgs::PoseStamped goal){
	  vector<geometry_msgs::Pose> poses;
	  vector<string> ids;
	  geometry_msgs::Point point;
	  point.x = goal.pose.position.x;
	  point.y = goal.pose.position.y;
	  point.z = goal.pose.position.z;
	  opendrive::point2parkingSpaces(_roadmap, point, poses, ids);
	  if(ids.size() > 0) {
		  _parkingId = ids[0];
	  }
    return true;
}

bool MapEnvironment::update(const Point& p) {
	if (_route.size() == 0) return false;
	GlobalLaneId oldLaneId = _mapState.curLaneId;
	Point cp = p;
	if (_envLanes.size() > 0)
		_trans->xyz2sli(_selfId, cp, get_local_lanes(_envLanes, _mapState.curS - 10, _mapState.curS + 10), _mapState.curLaneId);
	if (_envLanes.size() == 0 || _mapState.curLaneId.id == "")
		_trans->xyz2sli(_selfId, cp, _route_lanes, _mapState.curLaneId);
	_mapState.outOfRoute = _mapState.curLaneId.id == "" ? true : false; // whether car is on the road
	if (_mapState.curLaneId.id == "") { // not on the road
		geometry_msgs::Point geoPoint;
		geoPoint.x = p.x, geoPoint.y = p.y, geoPoint.z = p.z;
		vector<ParkingSpace> parkings;
		vector<string> roadIDs;
		point2parkingSpaces(_roadmap, geoPoint, parkings, roadIDs);
		if (roadIDs.size() > 0) {
			GlobalLaneId laneId = get_nearest_lane((*_roadmap)[roadIDs[0]], parkings[0].get_s(), parkings [0].get_pose().position);
			if (_globalMapLanes.find(laneId) != _globalMapLanes.end()) {
				cp.s = parkings[0].get_s();
				_mapState.curLaneId = laneId;
			}
		}
	}
	if (_mapState.curLaneId.id == "") { // not on the driving space
		// _envLanes.clear();
		if (_selfId < 0) fillMapGrid(_globalEnvLanes, p, _grid, _parkingId);
		return false;
	}
	updateCurState(cp);

	int longestIndex;
	double longestRemain;
	if (_envLanes.size() > 0) {
		getLongestRemain(longestIndex, longestRemain);
		if (longestRemain < Front_Min_Extend_Length && !_line_end || !(oldLaneId == _mapState.curLaneId)) {
			_line_end = false;
			get_local_envlanes(_roadmap, _globalEnvLanes, _junctionEnvs, _envLanes, _mapLanes, _mapState.curLaneId, _mapState.curS);
			_local_lanes = get_laneIds();
			// if (_selfId < 0) print(_envLanes);
		}
		getLongestRemain(longestIndex, longestRemain);
		if (longestRemain < Front_Min_Extend_Length) _line_end = true;
	}
	else {
		get_local_envlanes(_roadmap, _globalEnvLanes, _junctionEnvs, _envLanes, _mapLanes, _mapState.curLaneId, _mapState.curS);
		_local_lanes = get_laneIds();
		// if (_selfId < 0) print(_envLanes);
	}
	if (_selfId < 0) fillMapGrid(_envLanes, p, _grid, _parkingId);
	return true;
}

void MapEnvironment::updateCurState(const Point& p) {
	int minI = 0;
	double minD = 1e10;
	const vector<MapInfo>& mapInfos = _globalMapLanes[_mapState.curLaneId];
	for (int i = 0; i < mapInfos.size(); ++i) {
		double tempCurS = mapInfos[i].map2envS(p.s);
		double dist = fabs(tempCurS - _mapState.curS);
		if (dist < minD) {
			minI = i;
			minD = dist;
		}
	}

	_mapState.laneChange = 0;
	if (mapInfos[minI].globalEnvId - _mapState.curEnvId != 0) {
		int index = 0;
		const EnvLane* ptr = (*this)[mapInfos[minI].globalEnvId];
		if (ptr && ptr->get_tagindex('T', index))
			_mapState.laneChange = index;
	}
	_mapState.curEnvId = mapInfos[minI].globalEnvId;
	_mapState.curS = mapInfos[minI].map2envS(p.s);
	_mapState.curRemain = _globalEnvLanes[_mapState.curEnvId].get_endEnvS() - _mapState.curS;
}

void MapEnvironment::print(const map<int, EnvLane>& envLanes) const {
	map<int, EnvLane>::const_iterator iter = envLanes.begin();
	for ( ; iter != envLanes.end(); ++iter) {
		iter->second.print();
	}
	cout << "--------------------" << endl;
}

bool MapEnvironment::XYZ2SIL(Point &p, int obsId, int& envId) const {
	GlobalLaneId result;
	if (obsId == -1 && _selfId == -1)
		_trans->xyz2sli(obsId, p, get_local_lanes(_envLanes, _mapState.curS - 10, _mapState.curS + 10), result);
	else
		_trans->xyz2sli(obsId, p, _local_lanes, result);
	if (result.id == "" || _mapLanes.find(result) == _mapLanes.end()) return false;
	const MapInfo* mapInfo = &(_mapLanes.find(result)->second[0]);
	envId = mapInfo->globalEnvId;
	mapInfo->get_envSLT(p);
	return true;
}

Point MapEnvironment::mapSL2XYZ(string &roadId, double s, double l) const {
	opendrive::Point op(0, 0, 0, s, l, 0);
	sl2xyz((*_roadmap)[roadId], op);
	return op;
}

double MapEnvironment::SI2L(double s, const EnvLane* envLane) const {
	return SI2XYZ(s, envLane).l;
}

Point MapEnvironment::SI2XYZ(double s, const EnvLane* envLane) const {
	vector<double> ss(1, s);
	vector<Point> tmp = SI2XYZ(ss, envLane);
	if (tmp.size() > 0) return tmp[0];
	else ROS_ERROR("SI2XYZ convert error");
	return Point(0, 0, 0, 0, 0, 0, 0);
}

vector<Point> MapEnvironment::SI2XYZ(const vector<double>& ss, const EnvLane* envLane) const {
	vector<double> dls(ss.size(), 0.0);
	return SIdL2XYZ(ss, dls, envLane);
}

Point MapEnvironment::SIdL2XYZ(double s, double dl, const EnvLane* envLane) const {
	vector<double> ss(1, s);
	vector<double> dls(1, dl);
	vector<Point> tmp = SIdL2XYZ(ss, dls, envLane);
	if (tmp.size() > 0) return tmp[0];
	else ROS_ERROR("SI2XYZ convert error");
	return Point(0, 0, 0, 0, 0, 0, 0);
}

vector<Point> MapEnvironment::SIdL2XYZ(const vector<double>& ss, double dl, const EnvLane* envLane) const {
	vector<double> dls(ss.size(), dl);
	return SIdL2XYZ(ss, dls, envLane);
}

// inner: l < 0, outer: l > 0
vector<Point> MapEnvironment::SIdL2XYZ(const vector<double>& ss, const vector<double>& dls, const EnvLane* envLane) const {
	vector<Point> result;
	if (envLane == NULL) envLane = getCurEnvLane();
	const MapInfo* mapInfo = NULL;
	const Lane* lane = NULL;
	for (int i = 0; i < ss.size(); ++i) {
		if (ss[i] < envLane->get_envSOffset() || ss[i] > envLane->get_endEnvS()) continue;
		if (mapInfo == NULL || ss[i] > mapInfo->envSOffset + mapInfo->length) {
			mapInfo = envLane->get_mapInfo(ss[i]);
			lane = (*_roadmap)[*(envLane->get_globalId(ss[i]))];
		}
		Point p;
		p.s = mapInfo->env2mapS(ss[i]);
		double mapS = mapInfo->env2mapS(ss[i]);
		double mapDL;
		if (!mapInfo->reversed && (Drive_Direction > 0 || mapInfo->reversed && Drive_Direction < 0)) mapDL = -dls[i];
		else mapDL = dls[i];
		result.push_back(sdl2point(lane, mapS, mapDL));
		mapInfo->get_envSLT(result.back());
	}
	return result;
}




Point MapEnvironment::SI2XYZ(const GlobalLaneId &id, double s) const {
	const Lane* lane = (*_roadmap)[id];
	opendrive::Point p = s2center_point(lane, s);
	return p;
}

Point MapEnvironment::SIL2XYZ(const GlobalLaneId &id, double s, double l) const {
	const Lane* lane = (*_roadmap)[id];
	opendrive::Point p = sl2point(lane, s, l);
	return p;
}

void MapEnvironment::getLongestRemain(int& index, double &remain) const {
	index = 0;
	remain = _mapState.curRemain;
	for (map<int, EnvLane>::const_iterator i = _envLanes.begin(); i != _envLanes.end(); ++i) {
		int tindex = 0;
		if ( !(i->second.get_tagindex('T', tindex) )) {
			continue;
		}

	if (i->second.get_endEnvS() - _mapState.curS > remain) {
			remain = i->second.get_endEnvS() - _mapState.curS;
			index = tindex;
		}
	}
}

void MapEnvironment::getConflictS(vector<int>& envIds, vector<double>& selfS, vector<double>& otherS) {
	envIds.clear(), selfS.clear(), otherS.clear();
	GlobalLaneId inLane, outLane;
	string jid = get_self_junction(inLane, outLane);
	if (jid == "") return;
	_junctionEnvs[jid].get_conflictS(inLane, outLane, envIds, selfS, otherS);
	const EnvLane* envLane = _junctionEnvs[jid].get_envLane(inLane, outLane);
	double ds = _envLanes[_mapState.curEnvId].get_junctionStartS() - envLane->get_junctionStartS();
	for (int k = 0; k < selfS.size(); ++k) selfS[k] += ds;
}

nav_msgs::OccupancyGrid& MapEnvironment::getGrid() {
	return _grid;
}

string MapEnvironment::get_self_junction(GlobalLaneId &inLane, GlobalLaneId &outLane) const {
	pair<double, double> startEndS;
	const EnvLane& envLane = *getCurEnvLane();
	if (!getCurEnvLane()->get_nearest_junctionS(_mapState.curS, startEndS)) return "";
	const vector<GlobalLaneId>& globalIds = envLane.get_globalIds();
	const GlobalLaneId* pInLane = envLane.get_globalId(startEndS.first - 0.1);
	const GlobalLaneId* pJLane = envLane.get_globalId(startEndS.first + 0.1);
	const GlobalLaneId* pOutLane = envLane.get_globalId(startEndS.second + 0.1);
	if (!pInLane || !pOutLane || pInLane == pOutLane) return "";
	inLane = *pInLane, outLane = *pOutLane;
	return (*_roadmap)[(*pJLane).id]->get_junction();
}

vector<GlobalLaneId> MapEnvironment::get_laneIds() const {
	vector<GlobalLaneId> result;
	for (map<int, EnvLane>::const_iterator i = _envLanes.begin(); i != _envLanes.end(); ++i) {
		if (i->second.is_Tlane() || i->second.is_Rlane() || i->second.is_Flane())
			result.insert(result.end(), i->second.get_globalIds().begin(), i->second.get_globalIds().end());
	}
	return result;
}

}
