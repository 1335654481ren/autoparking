/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: roadmap_fun.cpp
*   Author  : lubing.han
*   Date    : 2017-05-03
*   Describe:
*
********************************************************/
#include "roadmap_fun.h"
#include "map_api_parameters.h"
#include <cmath>
using namespace std;

namespace opendrive {

bool get_junction_env(const RoadMap* roadMap, string junctionId, JunctionEnv& junctionEnv, int& globalEnvId);
void append_markers(const RoadMap* roadMap, EnvLane& envLane);
void extend_envlane(const RoadMap* roadMap, const vector<string>& route,
	const vector<bool>& reversed, const vector<double>& routeS, map<int, EnvLane>& envLanes,
	map<GlobalLaneId, vector<MapInfo> >& mapLanes, const Lane* lane, double envS, int index);
bool check_exist(map<GlobalLaneId, vector<MapInfo> >& mapLanes, GlobalLaneId id, double s);
void get_reversed(const RoadMap* roadMap, const vector<string>& route, vector<bool>& reversed);

bool get_global_envlanes(const RoadMap* roadMap, const vector<string>& route,
	vector<bool>& reversed, vector<double>& routeS, map<int, EnvLane>& envLanes,
	map<GlobalLaneId, vector<MapInfo> >& mapLanes) {
	get_reversed(roadMap, route, reversed);
	routeS.resize(route.size());
	double sumS = 0;
	for (int i = 0; i < route.size(); ++i) {
		routeS[i] = sumS;
		sumS += (*roadMap)[route[i]]->get_length();
	}
	envLanes.clear();
	mapLanes.clear();
	GlobalLaneId globalId;
	const LaneSection* section;
	if (reversed[0]) section = (*roadMap)[route[0]]->back();
	else section = (*roadMap)[route[0]]->front();
	for (const Lane* l = section->front(); l != NULL; l = section->next(l)) {
		if (l->get_id() != 0)
			extend_envlane(roadMap, route, reversed, routeS, envLanes, mapLanes, l, 0, 0);
	}
	int laneIndex = 0;
	while (laneIndex < envLanes.size()) {
		EnvLane envLane = envLanes[laneIndex];
		const vector<GlobalLaneId>& globalIds = envLane.get_globalIds();
		const vector<MapInfo>& mapInfos = envLane.get_mapInfos();
		for (int i = 0; i < globalIds.size(); ++i) {
			double envS = mapInfos[i].envSOffset;
			int roadIndex = 0;
			for ( ; roadIndex + 1 < route.size(); ++ roadIndex)
				if (routeS[roadIndex] - 0.1 < envS && envS < routeS[roadIndex + 1] - 0.1) break;
			section = (*((*roadMap)[globalIds[i].id]))[globalIds[i].sectionId];
			for (const Lane* l = section->front(); l != NULL; l = section->next(l)) {
				if (l->get_id() != 0)
					extend_envlane(roadMap, route, reversed, routeS, envLanes, mapLanes, l, envS, roadIndex);
			}
		}
		++laneIndex;
	}
	return true;
}

bool get_global_junctions(const RoadMap* roadMap, const map<int, EnvLane>& globalEnvLanes,
	map<string, JunctionEnv>& junctionLanes) {
	junctionLanes.clear();
	map<int, EnvLane>::const_iterator envLanesIter = globalEnvLanes.begin();
	int globalEnvId = globalEnvLanes.size();
        for ( ; envLanesIter != globalEnvLanes.end(); ++envLanesIter) {
		const vector<GlobalLaneId>& globalIds = envLanesIter->second.get_globalIds();
		for (int i = 0; i < globalIds.size(); ++i) {
			string junctionId = (*roadMap)[globalIds[i].id]->get_junction();
			if (junctionId == "-1") continue;
			const Junction* junction = roadMap->find_junction(junctionId);
			if (junctionLanes.find(junctionId) != junctionLanes.end()) continue;
                        JunctionEnv junctionEnv;
			if (get_junction_env(roadMap, junctionId, junctionEnv, globalEnvId))
                                junctionLanes[junctionId] = junctionEnv;
                }
	}
	return true;
}

bool get_local_envlanes(const RoadMap* roadMap, const map<int, EnvLane>& globalEnvLanes,
	const map<string, JunctionEnv> junctionEnvs, map<int, EnvLane>& obsEnvLanes,
	map<GlobalLaneId, vector<MapInfo> >& obsMapLanes, GlobalLaneId curId, double curS) {
	obsEnvLanes.clear();
	obsMapLanes.clear();
	map<GlobalLaneId, int> mapObsEnv;
	int FTagCount = 0;
	double minS = curS - Back_Extend_Length, maxS = curS + Front_Extend_Length;
	map<int, EnvLane>::const_iterator envLanesIter = globalEnvLanes.begin();
	for ( ; envLanesIter != globalEnvLanes.end(); ++envLanesIter) {
		int globalEnvId = envLanesIter->second.get_globalEnvId();
		const vector<GlobalLaneId>& globalIds = envLanesIter->second.get_globalIds();
		const vector<MapInfo>& mapInfos = envLanesIter->second.get_mapInfos();
		if (envLanesIter->second.get_envSOffset() <= curS && envLanesIter->second.get_endEnvS() >= curS) {
			obsEnvLanes[globalEnvId] = EnvLane(envLanesIter->second, minS, maxS);
			const vector<GlobalLaneId>& tempGlobalIds = obsEnvLanes[globalEnvId].get_globalIds();
			for (int i = 0; i < tempGlobalIds.size(); ++i) mapObsEnv[tempGlobalIds[i]] = envLanesIter->first;
			for (int i = 0; i < globalIds.size(); ++i) {
				if (obsMapLanes.find(globalIds[i]) == obsMapLanes.end()) obsMapLanes[globalIds[i]] = vector<MapInfo>();
				obsMapLanes[globalIds[i]].push_back(mapInfos[i]);
			}
		}
		else if (envLanesIter->second.get_envSOffset() > curS && envLanesIter->second.get_envSOffset() <= maxS) {
			obsEnvLanes[globalEnvId] = EnvLane(envLanesIter->second, minS, maxS);
			obsEnvLanes[globalEnvId].append_tag(make_pair('F', FTagCount++));
			obsEnvLanes[globalEnvId].set_type("driving");
			const vector<GlobalLaneId>& tempGlobalIds = obsEnvLanes[globalEnvId].get_globalIds();
			for (int i = 0; i < tempGlobalIds.size(); ++i) mapObsEnv[tempGlobalIds[i]] = envLanesIter->first;
			for (int i = 0; i < globalIds.size(); ++i) {
				if (obsMapLanes.find(globalIds[i]) == obsMapLanes.end()) obsMapLanes[globalIds[i]] = vector<MapInfo>();
				obsMapLanes[globalIds[i]].push_back(mapInfos[i]);
			}
		}
	}

	// add T and R tags
	int curEnvId = mapObsEnv[curId];
	bool curReversed = obsEnvLanes[curEnvId].get_mapInfo(curS)->reversed;
	const LaneSection* section = (*((*roadMap)[curId.id]))[curId.sectionId];
	int addLaneId, reverseLaneId, TlaneId, RTagCount = 0, TTagCount = 0;
	if ((2 * curReversed - 1) * Drive_Direction * curId.laneId > 0) addLaneId = 2 * (curId.laneId < 0) - 1;
	else addLaneId = 2 * (curId.laneId > 0) - 1;
	reverseLaneId = addLaneId;
	const Lane* lane = (*section)[reverseLaneId];
	// R tags
	while (lane) {
		if (mapObsEnv.find(lane->get_globalId()) == mapObsEnv.end()) break;
		int envId = mapObsEnv[lane->get_globalId()];
		obsEnvLanes[envId].append_tag(make_pair('R', RTagCount++));
		obsEnvLanes[envId].set_type(lane->get_type());
		reverseLaneId += addLaneId;
		lane = (*section)[reverseLaneId];
	}
	addLaneId *= -1;
	TlaneId = curId.laneId;
	lane = (*section)[TlaneId];
	// T tags
	while (lane) {
		if (mapObsEnv.find(lane->get_globalId()) == mapObsEnv.end()) break;
		int envId = mapObsEnv[lane->get_globalId()];
		obsEnvLanes[envId].append_tag(make_pair('T', TTagCount++));
		obsEnvLanes[envId].set_type(lane->get_type());
		TlaneId += addLaneId;
		if (TlaneId == 0) TlaneId += addLaneId;
		lane = (*section)[TlaneId];
	}
	if (curId.laneId * addLaneId > 0) {
		addLaneId *= -1;
		TlaneId = curId.laneId + addLaneId;
		TTagCount = -1;
		lane = (*section)[TlaneId];
		while (lane && TlaneId != 0) {
			if (mapObsEnv.find(lane->get_globalId()) == mapObsEnv.end()) break;
			int envId = mapObsEnv[lane->get_globalId()];
			obsEnvLanes[envId].append_tag(make_pair('T', TTagCount--));
			obsEnvLanes[envId].set_type(lane->get_type());
			TlaneId += addLaneId;
			lane = (*section)[TlaneId];
		}
	}

	// add Junction tag
	bool got_junction = false;
	for (map<int, EnvLane>::iterator i = obsEnvLanes.begin(); i != obsEnvLanes.end(); ++i) {
		EnvLane& curLane = i->second;
		if (!curLane.is_Tlane()) continue;
		const vector<GlobalLaneId>& globalIds = curLane.get_globalIds();
		const vector<MapInfo>& mapInfos = curLane.get_mapInfos();
		int jIndex = 0;
		for ( ; jIndex < globalIds.size(); ++jIndex) {
			if ((*roadMap)[globalIds[jIndex].id]->get_junction() != "-1"
				&& mapInfos[jIndex].envSOffset + mapInfos[jIndex].length > curS)
				break;
		}
		if (jIndex == 0 || jIndex + 1 >= globalIds.size()) continue;
		string jId = (*roadMap)[globalIds[jIndex].id]->get_junction();
		if (junctionEnvs.find(jId) == junctionEnvs.end()) continue;
		int mode = junctionEnvs.find(jId)->second.get_mode(globalIds[jIndex - 1], globalIds[jIndex + 1]);
		if (mode < 0) continue;
		curLane.append_tag(make_pair('J', mode));
		curLane.set_junctionStartS(mapInfos[jIndex].envSOffset);
		curLane.set_junctionEndS(curLane.get_junctionStartS() + mapInfos[jIndex].length);
		if  (!got_junction) {
			const map<int, EnvLane>& jLanes = junctionEnvs.find(jId)->second.get_junctionLanes();
			for (map<int, EnvLane>::const_iterator i = jLanes.begin(); i != jLanes.end(); ++i) {
				obsEnvLanes[i->first] = i->second;
				obsEnvLanes[i->first].set_type("driving");
			}
			got_junction = true;
		}
	}

	return true;
}

bool get_junction_env(const RoadMap* roadMap, string junctionId, JunctionEnv& junctionEnv, int& globalEnvId) {
	const Junction* junction = roadMap->find_junction(junctionId);
	if (!junction) return false;
	vector<string> roads = junction->get_connectingRoads();
	if (roads.size() < 2) return false;

	// get basic info
	vector<GlobalLaneId> fromLanes, viaLanes, toLanes;
	vector<double> fromThetas, toThetas;
	vector<int> fromGroup, toGroup;
	for (int i = 0; i < roads.size(); ++i) {
		if ((*roadMap)[roads[i]]->front() != (*roadMap)[roads[i]]->back()) {
			cout << "error in get_organized_junction: more than one section in a junction road" << endl;
			return false;
		}
		const LaneSection* section = (*roadMap)[roads[i]]->front();
		for (const Lane* lane = section->front(); lane != NULL; lane = section->next(lane)) {
			if (lane->get_id() == 0 || lane->get_predecessors().size() != 1 || lane->get_successors().size() != 1) continue;
			viaLanes.push_back(lane->get_globalId());
			if (Drive_Direction * lane->get_id() < 0) {
				fromLanes.push_back(lane->get_predecessors()[0]);
				toLanes.push_back(lane->get_successors()[0]);
				fromThetas.push_back(lane->get_center().front().theta + M_PI);
				toThetas.push_back(lane->get_center().back().theta);
			}
			else {
				fromLanes.push_back(lane->get_successors()[0]);
				toLanes.push_back(lane->get_predecessors()[0]);
				fromThetas.push_back(lane->get_center().back().theta);
				toThetas.push_back(lane->get_center().front().theta + M_PI);	
			}
		}
	}
	double baseTheta = fromThetas[0];
	for (int i = 0; i < fromThetas.size(); ++i) {
		fromGroup.push_back(int(16 + (fromThetas[i] - baseTheta + M_PI / 4) / (M_PI / 2)) % 4);
		toGroup.push_back(int(16 + (toThetas[i] - baseTheta + M_PI / 4) / (M_PI / 2)) % 4);
	}
	junctionEnv.reset_env(globalEnvId, junctionId, fromLanes, viaLanes, toLanes, fromGroup, toGroup);
	return true;
}

void append_markers(const RoadMap* roadMap, EnvLane& envLane) {
	// envLane.print();
	const vector<MapInfo>& mapInfos = envLane.get_mapInfos();
	const vector<GlobalLaneId>& globalIds = envLane.get_globalIds();
	for (int i = 0; i < globalIds.size(); ++i) {
		const Lane* lane = (*roadMap)[globalIds[i]];
		// if (i > 0 && globalIds[i].id == globalIds[i - 1].id) continue;
		vector<Marker> markers = get_markers((*roadMap)[globalIds[i].id], globalIds[i], MarkerFilter(""));
		for (int j = 0; j < markers.size(); ++j) {
			if (markers[j].get_type() == "stopLine")
				envLane.append_stopLine(StopLine(lane, markers[j], mapInfos[i]));
			else if (markers[j].get_type() == "crosswalk")
				envLane.append_crossWalk(CrossWalk(lane, markers[j], mapInfos[i]));
			else if (markers[j].get_type() == "parkingSpace")
				envLane.append_parkingSpace(ParkingSpace(lane, markers[j], mapInfos[i]));
			else if (markers[j].get_type() == "normalObject")
				envLane.append_normalObject(NormalObject(lane, markers[j], mapInfos[i]));
		}
	}
}

void append_laneChanges(const RoadMap* roadMap, EnvLane& envLane) {
	const vector<MapInfo>& mapInfos = envLane.get_mapInfos();
	const vector<GlobalLaneId>& globalIds = envLane.get_globalIds();
	for (int i = 0; i < globalIds.size(); ++i) {
		const LaneSection* section = (*((*roadMap)[globalIds[i].id]))[globalIds[i].sectionId];
		vector<pair<double, bool> > innerLaneChanges = get_laneChanges(section, globalIds[i].laneId, false, mapInfos[i].reversed);
		vector<pair<double, bool> > outerLaneChanges = get_laneChanges(section, globalIds[i].laneId, true, mapInfos[i].reversed);
		for (int j = 0; j < innerLaneChanges.size(); ++j) {
			innerLaneChanges[j].first = mapInfos[i].map2envS(innerLaneChanges[j].first);
			envLane.append_innerLaneChange(innerLaneChanges[j]);
		}
		for (int j = 0; j < outerLaneChanges.size(); ++j) {
			outerLaneChanges[j].first = mapInfos[i].map2envS(outerLaneChanges[j].first);
			envLane.append_outerLaneChange(outerLaneChanges[j]);
		}
	}
}

void extend_envlane(const RoadMap* roadMap, const vector<string>& route,
	const vector<bool>& reversed, const vector<double>& routeS, map<int, EnvLane>& envLanes,
	map<GlobalLaneId, vector<MapInfo> >& mapLanes, const Lane* lane, double envS, int index) {
	EnvLane envLane(envLanes.size(), envS);
	GlobalLaneId nowId = lane->get_globalId();
	if (nowId.id != route[index]) return;

	double junctionStartS = 0.0, junctionEndS;
	bool inJunction = false;
	while (!check_exist(mapLanes, nowId, envS)) {
		lane = (*roadMap)[nowId];
		string roadJunction = (*roadMap)[lane->get_globalId().id]->get_junction();
		if ((roadJunction == "-1" || roadJunction == "") && inJunction) {
			inJunction = false;
			junctionEndS = envLane.get_endEnvS();
			envLane.append_junctionS(make_pair(junctionStartS, junctionEndS));
		}
		else if (!(roadJunction == "-1" || roadJunction == "") && !inJunction) {
			inJunction = true;
			junctionStartS = envLane.get_endEnvS();
		}
		envLane.append_lane(lane, reversed[index], "forward");
		if (mapLanes.find(nowId) == mapLanes.end()) mapLanes[nowId] = vector<MapInfo>();
		mapLanes[nowId].push_back(envLane.get_mapInfos().back());

		vector<GlobalLaneId> nextIds;
		if (reversed[index]) nextIds = lane->get_predecessors();
		else nextIds = lane->get_successors();

		envS += lane->get_length();
		bool flag = true;
		for (int i = 0; i < nextIds.size(); ++i) {
			if (nextIds[i].id == route[index]) {
				nowId = nextIds[i];
				flag = false;
				break;
			}
			else if (index < route.size() && nextIds[i].id == route[index + 1]) {
				nowId = nextIds[i];
				++index;
				flag = false;
				break;
			}
		}
		if (flag) break;
	}
	if (inJunction) envLane.append_junctionS(make_pair(junctionStartS, envLane.get_endEnvS()));

	if (envLane.get_mapInfos().size() == 0) return;
	append_markers(roadMap, envLane);
	append_laneChanges(roadMap, envLane);
	envLanes[envLanes.size()] = envLane;
}

bool check_exist(map<GlobalLaneId, vector<MapInfo> >& mapLanes, GlobalLaneId id, double s) {
	if (mapLanes.find(id) == mapLanes.end()) return false;
	const vector<MapInfo>& mapInfos = mapLanes[id];
	for (int i = 0; i < mapInfos.size(); ++i) {
		if (fabs(mapInfos[i].envSOffset - s) < 0.1) return true;
	}
	return false;
}

void get_reversed(const RoadMap* roadMap, const vector<string>& route, vector<bool>& reversed) {
	reversed.resize(route.size());
	if (route.size() == 1) {
		reversed[0] = false;
		return;
	}
	for (int i = 0; i < route.size(); ++i) {
		string nowId, nextId;
		nowId = route[i];
		nextId = i + 1 == route.size() ? route[i - 1] : route[i + 1];	
		if ((*roadMap)[nowId]->get_relation(nextId) == RELATION::SUCCESSOR)
			reversed[i] = i + 1 == route.size() ? true : false;
		else
			reversed[i] = i + 1 == route.size() ? false : true;	
	}
}

// deprecated
void point2parkingSpaces(const RoadMap* roadMap, const geometry_msgs::Point &point,
	vector<geometry_msgs::Pose>& poses, vector<string>& ids) {
	poses.clear(), ids.clear();
	const Road* road = roadMap->front();
	while (road) {
		const vector<Marker>& markers = road->get_markers();
		for (int i = 0; i < markers.size(); ++i) {
			if (markers[i].get_type() != "parkingSpace") continue;
			const Lane* lane = (*((*road)[road->get_section_index(markers[i].get_s())]))[0];
			MapInfo mapInfo(0, GlobalLaneId(), false, 0.0, 0.0, lane->get_length());
			ParkingSpace parkingSpace(lane, markers[i], mapInfo);
			geometry_msgs::Pose pose = parkingSpace.get_pose();
			double theta = tf::getYaw(pose.orientation);
			double dx = point.x - pose.position.x;
			double dy = point.y - pose.position.y;
			double dl = dx * cos(theta) + dy * sin(theta);
			double dw = dx * sin(theta) - dy * cos(theta);
			if (fabs(dl) < 2.5 && fabs(dw) < 1.0) {
				poses.push_back(pose);
				ids.push_back(markers[i].get_id());
			}
		}
		road = roadMap->next(road);
	}
}

void point2parkingSpaces(const RoadMap* roadMap, const geometry_msgs::Point &point, vector<ParkingSpace>& parkings) {
	vector<string> roadIDs;
	point2parkingSpaces(roadMap, point, parkings, roadIDs);
}

void point2parkingSpaces(const RoadMap* roadMap, const geometry_msgs::Point &point,
	vector<ParkingSpace>& parkings, vector<string>& roadIDs) {
	parkings.clear(), roadIDs.clear();
	const Road* road = roadMap->front();
	while (road) {
		const vector<Marker>& markers = road->get_markers();
		for (int i = 0; i < markers.size(); ++i) {
			if (markers[i].get_type() != "parkingSpace") continue;
			const Lane* lane = (*((*road)[road->get_section_index(markers[i].get_s())]))[0];
			MapInfo mapInfo(0, GlobalLaneId(), false, 0.0, 0.0, lane->get_length());
			ParkingSpace parkingSpace(lane, markers[i], mapInfo);
			if (parkingSpace.in_parkingSpace(point)) {
				parkings.push_back(parkingSpace);
				roadIDs.push_back(road->get_id());
			}
		}
		road = roadMap->next(road);
	}
}

}
