/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: router.cpp
*   Author  : lubing.han
*   Date    : 2017-03-06
*   Describe:
*
********************************************************/
#include "router.h"
#include <algorithm>
using namespace std;

namespace opendrive {

RouterNode::RouterNode() {
	init();
}

RouterNode::RouterNode(const GlobalLaneId &id): globalId(id) {
	init();
}

RouterNode::~RouterNode() {

}

void RouterNode::init() {
	visited = false;
	closed = false;
	s = 1e10;
	last = NULL;
}

double RouterNode::dist(const RouterNode* other) {
	if (other->globalId.id == globalId.id && other->globalId.sectionId == globalId.sectionId)
		return 0.0;
	return len;
}

GlobalLaneId RouterNode::get_id() {
	return globalId;
}

Router::Router() {
	string filepath;
	_roadMap = RoadMap::get_instance();
}

Router::~Router() {

}

vector<string> Router::getRoute(Point startP, Point endP) const {
	vector<GlobalLaneId> laneIds;
	const Lane* lane = _roadMap->front3();
	for ( ; lane != NULL; lane = _roadMap->next(lane))
			laneIds.push_back(lane->get_globalId());
	CoordinatesTrans* trans = CoordinatesTrans::getInstance();
	trans->update();

	GlobalLaneId startLaneId, endLaneId;
	trans->xyz2sli(-100, startP, laneIds, startLaneId);
	trans->xyz2sli(-100, endP, laneIds, endLaneId);

	if (startLaneId.id == "") {
		vector<ParkingSpace> parkingSpaces;
		vector<string> roadIDs;
		geometry_msgs::Point geoStartP;
		geoStartP.x = startP.x, geoStartP.y = startP.y, geoStartP.z = startP.z;
		point2parkingSpaces(_roadMap, geoStartP, parkingSpaces, roadIDs);
		if (parkingSpaces.size() == 1) {
			startLaneId.id = roadIDs[0];
			startLaneId.sectionId = -1;
		}
	}

	if (endLaneId.id == "") {
		vector<ParkingSpace> parkingSpaces;
		vector<string> roadIDs;
		geometry_msgs::Point geoEndP;
		geoEndP.x = endP.x, geoEndP.y = endP.y, geoEndP.z = endP.z;
		point2parkingSpaces(_roadMap, geoEndP, parkingSpaces, roadIDs);
		if (parkingSpaces.size() == 1) {
			endLaneId.id = roadIDs[0];
			endLaneId.sectionId = -1;
		}
	}
	if (startLaneId.id == "" || endLaneId.id == "") return vector<string>();
	return getRoute(startLaneId, endLaneId);
}

vector<string> Router::getRoute(const GlobalLaneId& fromId, const GlobalLaneId& toId,
	double startS, bool enableLaneChange, double endS) const {
	vector<GlobalLaneId> firstLanes = getLanes(fromId, startS, enableLaneChange, true);
	vector<GlobalLaneId> lastLanes = getLanes(toId, endS, true, false);
	vector<string> route;
	if (firstLanes.size() * lastLanes.size() == 0) return route;
	// cout << "Start Lanes:" << endl;
	// for (int i = 0; i < firstLanes.size(); ++i)
	// 	firstLanes[i].print();
	// cout << "End Lanes:" << endl;
	// for (int i = 0; i < lastLanes.size(); ++i)
	// 	lastLanes[i].print();

	map<GlobalLaneId, RouterNode> nodeGraph = constructGraph(firstLanes, lastLanes);
	vector<GlobalLaneId> laneRoute = shortestPath(&nodeGraph[GlobalLaneId("__start__", 0, 0)], &nodeGraph[GlobalLaneId("__end__", 0, 0)]);
	vector<string> roadRoute;
	if (laneRoute.size() < 2) return roadRoute;

	// cout << "Lane Route:" << endl;
	// fromId.print();
	roadRoute.push_back(fromId.id);
	for (int i = 1; i < laneRoute.size() - 1; ++i) {
		if (laneRoute[i].id != roadRoute.back())
			roadRoute.push_back(laneRoute[i].id);
		// laneRoute[i].print();
	}
	// toId.print();
	if (roadRoute.back() != toId.id)
		roadRoute.push_back(toId.id);
	return roadRoute;
}

vector<GlobalLaneId> Router::getLanes(const GlobalLaneId& laneId, double s, bool enableLaneChange, bool isStart) const {
	vector<GlobalLaneId> laneIds;
	const Road* road = (*_roadMap)[laneId.id];
	if (road == NULL) return laneIds;
	
	if (laneId.sectionId < 0) {
		// Any lanes in road
		for (const Lane* lane = road->front2(); lane != NULL; lane = road->next(lane))
			if (lane->get_type() == "driving" && lane->get_id() != 0)
				laneIds.push_back(lane->get_globalId());
	}
	else {
		const LaneSection* section = (*road)[laneId.sectionId];
		if (!section) return laneIds;
		if (laneId.laneId == 0 || laneId.laneId == INT_MAX) {
			// Any lanes in section
			for (const Lane* lane = section->front(); lane != NULL; lane = section->next(lane))
				if (lane->get_type() == "driving" && lane->get_id() != 0)
					laneIds.push_back(lane->get_globalId());
		}
		else {
			const Lane* lane = (*section)[laneId.laneId];
			if (!lane || lane->get_type() != "driving" || lane->get_id() == 0) return laneIds;
			if (s <= 0) {
				// Specific lane
				laneIds.push_back(lane->get_globalId());
			}
			else {
				if (enableLaneChange) {
					// Possible next lanes
					vector<const Lane*> lanes;
					set<GlobalLaneId> laneIdSet;
					lanes.push_back(lane);
					double ls = s;
					const Lane* llane = section->get_inner_neighbour(lane->get_id());
					for ( ; llane != NULL; llane = section->get_inner_neighbour(llane->get_id()))
						lanes.push_back(llane);
					double rs = s;
					const Lane* rlane = section->get_outer_neighbour(lane->get_id());
					for ( ; rlane != NULL; rlane = section->get_outer_neighbour(rlane->get_id()))
						lanes.push_back(rlane);
					for (int i = 0; i < lanes.size(); ++i) {
						vector<GlobalLaneId> tmpLaneIds;
						if (lanes[i]->get_id() * (isStart ? 1 : -1) * Drive_Direction > 0)
							tmpLaneIds = lanes[i]->get_predecessors();
						else if (lanes[i]->get_id() * (isStart ? 1 : -1) * Drive_Direction  < 0)
							tmpLaneIds = lanes[i]->get_successors();
						for (int i = 0; i < tmpLaneIds.size(); ++i)
							laneIdSet.insert(tmpLaneIds[i]);
					}
					for (set<GlobalLaneId>::iterator iter = laneIdSet.begin(); iter != laneIdSet.end(); ++iter)
						laneIds.push_back(*iter);
				}
				else {
					// Next lanes
					if (lane->get_id() * (isStart ? 1 : -1) * Drive_Direction > 0)
						laneIds = lane->get_predecessors();
					else if (lane->get_id() * (isStart ? 1 : -1) * Drive_Direction < 0)
						laneIds = lane->get_successors();
				}
			}
		}
	}
	return laneIds;
}

map<GlobalLaneId, RouterNode> Router::constructGraph(
	const vector<GlobalLaneId>& startLanes, const vector<GlobalLaneId>& endLanes) const {
	map<GlobalLaneId, RouterNode> laneGraph;
	// init graph
	GlobalLaneId startId = GlobalLaneId("__start__", 0, 0);
	laneGraph[startId] = RouterNode(startId);
	for (const Lane* lane = _roadMap->front3(); lane != NULL; lane = _roadMap->next(lane)) {
		if (lane->get_id() == 0) continue;
		laneGraph[lane->get_globalId()] = RouterNode(lane->get_globalId());
	}
	GlobalLaneId endId = GlobalLaneId("__end__", 0, 0);
	laneGraph[endId] = RouterNode(endId);
	// init nodes
	RouterNode &startNode = laneGraph[startId];
	startNode.len = 0.0;
	for (int i = 0; i < startLanes.size(); ++i) {
		if (laneGraph.find(startLanes[i]) != laneGraph.end())
			startNode.nexts.push_back(&laneGraph[startLanes[i]]);
	}

	for (const Lane* lane = _roadMap->front3(); lane != NULL; lane = _roadMap->next(lane)) {
		if (lane->get_id() == 0) continue;
		GlobalLaneId globalId = lane->get_globalId();
		RouterNode &rn = laneGraph[globalId];
		rn.len = lane->get_length();
		if (lane->get_inner_neighbour().size() > 0 && laneGraph.find(lane->get_inner_neighbour()[0]) != laneGraph.end())
			rn.nexts.push_back(&laneGraph[lane->get_inner_neighbour()[0]]);
		if (lane->get_outer_neighbour().size() > 0 && laneGraph.find(lane->get_outer_neighbour()[0]) != laneGraph.end())
			rn.nexts.push_back(&laneGraph[lane->get_outer_neighbour()[0]]);
		vector<GlobalLaneId> nexts;
		if (lane->get_id() * Drive_Direction < 0) nexts = lane->get_successors();
		else nexts = lane->get_predecessors();
		for (int i = 0; i < nexts.size(); ++i) {
			if (laneGraph.find(nexts[i]) != laneGraph.end())
				rn.nexts.push_back(&laneGraph[nexts[i]]);
		}
	}


	RouterNode* endNode = &laneGraph[endId];
	endNode->len = 0.0;
	for (int i = 0; i < endLanes.size(); ++i) {
		if (laneGraph.find(endLanes[i]) != laneGraph.end())
			laneGraph[endLanes[i]].nexts.push_back(endNode);
	}

	return laneGraph;
}

vector<GlobalLaneId> Router::shortestPath(RouterNode* startNode, RouterNode* endNode) const {
	vector<RouterNode*> visitedNodes;
	vector<GlobalLaneId> result;
	startNode->s = 0.0;
	startNode->visited = true;
	visitedNodes.push_back(startNode);
	while (visitedNodes.size() > 0) {
		vector<RouterNode*>::iterator iter;
		double d = 1e10;
		for (vector<RouterNode*>::iterator i = visitedNodes.begin(); i != visitedNodes.end(); ++i) {
			if ((*i)->s < d) {
				d = (*i)->s;
				iter = i;
			}
		}
		RouterNode* nowNode = *iter;
		visitedNodes.erase(iter);

		for (vector<RouterNode*>::iterator nextIter = nowNode->nexts.begin(); nextIter != nowNode->nexts.end(); ++nextIter) {
			if ((*nextIter)->closed) continue;
			double news = nowNode->s + nowNode->dist(*nextIter);
			if (!(*nextIter)->visited) {
				(*nextIter)->visited = true;
				(*nextIter)->s = news;
				(*nextIter)->last = nowNode;
				visitedNodes.push_back(*nextIter);
			}
			else {
				if ((*nextIter)->s > news) {
					(*nextIter)->s = news;
					(*nextIter)->last = nowNode;
				}
			}
		}
		nowNode->closed = true;
		if (nowNode == endNode) break;
	}
	RouterNode* rn = endNode;
	while (rn) {
		result.push_back(rn->get_id());
		rn = rn->last;
	}
	reverse(result.begin(), result.end());
	if (result.size() < 2) result.clear();
	return result;
}

}
