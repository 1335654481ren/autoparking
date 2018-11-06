/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: roadmap.cpp
*   Author  : lubing.han
*   Date    : 2016-12-07
*   Describe:
*
********************************************************/

#include "roadmap.h"
#include <limits.h>
#include <algorithm>
using namespace std;

namespace opendrive
{

#define RESET "\033[0m"
#define GREEN "\033[32m"
#define YELLOW "\033[33m"

RoadMap::RoadMap() {
	_initialized = false;
}

RoadMap::~RoadMap() {
}

const Road* RoadMap::operator[](const string &s) const {
	map<string, Road>::const_iterator iter = _roadmap.find(s);
	return iter == _roadmap.end() ? NULL : &iter->second;
}

const Lane* RoadMap::operator[](const GlobalLaneId &globallanid) const {
	const Road* road = (*this)[globallanid.id];
	return !road ? NULL : (*road)[globallanid];
}

const Road* RoadMap::front() const {
	map<string, Road>::const_iterator iter = _roadmap.begin();
	return iter == _roadmap.end() ? NULL : &iter->second;
}

const Lane* RoadMap::front3() const {
	const Road* road = this->front();
	return !road ? NULL : road->front2();
}

const Road* RoadMap::next(const Road* road) const {
	map<string, Road>::const_iterator iter = _roadmap.find(road->get_id());
	return (iter == _roadmap.end() || ++iter == _roadmap.end()) ? NULL : &iter->second;
}

const Lane* RoadMap::next(const Lane* lane) const {
	const Road* road = (*this)[lane->get_globalId().id];
	if (road->next(lane)) return road->next(lane);
	return !next(road) ? NULL : next(road)->front2();
}

Road* RoadMap::operator[](const string &s) {
	map<string, Road>::iterator iter = _roadmap.find(s);
	return iter == _roadmap.end() ? NULL : &iter->second;
}

Lane* RoadMap::operator[](const GlobalLaneId &globallanid) {
	Road* road = (*this)[globallanid.id];
	return !road ? NULL : (*road)[globallanid];
}

Road* RoadMap::front() {
	map<string, Road>::iterator iter = _roadmap.begin();
	return iter == _roadmap.end() ? NULL : &iter->second;
}

Lane* RoadMap::front3() {
	Road* road = this->front();
	return !road ? NULL : road->front2();
}

Road* RoadMap::next(const Road* road) {
	map<string, Road>::iterator iter = _roadmap.find(road->get_id());
	return (iter == _roadmap.end() || ++iter == _roadmap.end()) ? NULL : &iter->second;
}

Lane* RoadMap::next(const Lane* lane) {
	Road* road = (*this)[lane->get_globalId().id];
	if (road->next(lane)) return road->next(lane);
	return !next(road) ? NULL : next(road)->front2();
}

bool RoadMap::init() {
	if (_initialized) {
		cout << "RoadMap: init error: roadmap has been initialized" << endl;
		return false;
	}
	// init globalIds and points
	map<string, Road>::iterator iter = _roadmap.begin();
	for ( ; iter != _roadmap.end(); ++iter) {
		vector<LaneSection> &sections = iter->second.get_sections();
		for (int j = 0; j < sections.size(); ++j) {
			vector<Lane> &lanes = sections[j].get_lanes();
			for (int k = 0; k < lanes.size(); ++ k)
				lanes[k].set_globalId(iter->first, j, lanes[k].get_id());
		}
		iter->second.init_points();
	}
	init_road_relations();
	init_lane_relations();


	// Road* road = front();
	// while (road) {
	// 	cout << YELLOW << "Road ID: " << road->get_id() << RESET << endl;
	// 	cout << GREEN << "Predecessors:" << RESET << endl;
	// 	const vector<Link> &links = road->get_predecessors();
	// 	cout << "  ";
	// 	for (int i = 0; i < links.size(); ++i) cout << links[i].elementId << " ";
	// 	cout << endl;
	// 	cout << GREEN << "Successors:" << RESET << endl;
	// 	const vector<Link> &links2 = road->get_successors();
	// 	cout << "  ";
	// 	for (int i = 0; i < links2.size(); ++i) cout << links2[i].elementId << " ";
	// 	cout << endl;
	// 	road = next(road);
	// }

	// Lane* lane = front3();
	// while (lane) {
	// 	cout << YELLOW << "Lane ID: "; lane->get_globalId().print();

	// 	cout << GREEN << "Inner Neighbour: ";
	// 	if (lane->get_inner_neighbour().size() > 0) lane->get_inner_neighbour()[0].print();
	// 	else cout << endl;
	// 	cout << GREEN << "Outer Neighbour: ";
	// 	if (lane->get_outer_neighbour().size() > 0) lane->get_outer_neighbour()[0].print();
	// 	else cout << endl;
		
	// 	cout << GREEN << "Predecessors:" << RESET << endl;
	// 	const vector<GlobalLaneId> &pres = lane->get_predecessors();
	// 	for (int i = 0; i < pres.size(); ++i)
	// 		pres[i].print();
	// 	cout << GREEN << "Successors:" << RESET << endl;
	// 	const vector<GlobalLaneId> &sucs = lane->get_successors();
	// 	for (int i = 0; i < sucs.size(); ++i)
	// 		sucs[i].print();
		
	// 	lane = next(lane);
	// }
	// for (auto &i : _obstacles) i.second.print();
	_initialized = true;
	return true;
}

void RoadMap::init_road_relations() {
	// relations stored in road
	map<string, Road>::iterator iter = _roadmap.begin();
	for ( ; iter != _roadmap.end(); ++iter) {
		Link pre = iter->second.get_predecessor();
		Link suc = iter->second.get_successor();
		if (pre.elementType == "road" && (*this)[pre.elementId] != NULL) {
			iter->second.append_predecessor(pre);
			if (pre.contactPoint == "start")
				((*this)[pre.elementId])->append_predecessor(Link("road", iter->second.get_id(), "start"));
			else if (pre.contactPoint == "end")
				((*this)[pre.elementId])->append_successor(Link("road", iter->second.get_id(), "start"));
		}
		if (suc.elementType == "road" && (*this)[suc.elementId] != NULL) {
			iter->second.append_successor(suc);
			if (suc.contactPoint == "start")
				((*this)[suc.elementId])->append_predecessor(Link("road", iter->second.get_id(), "end"));
			else if (suc.contactPoint == "end")
				((*this)[suc.elementId])->append_successor(Link("road", iter->second.get_id(), "end"));
		}
	}
}

void RoadMap::init_lane_relations() {
	// get left and right neighbours
	for (Road* road = front(); road != NULL; road = next(road)) {
		for (LaneSection* section = road->front(); section != NULL; section = road->next(section)) {
			for (Lane* lane = section->front(); lane != NULL; lane = section->next(lane)) {
				GlobalLaneId globalId = lane->get_globalId();
				const Lane* llane = section->get_inner_neighbour(lane->get_id());
				if (llane) lane->append_inner_neighbour(GlobalLaneId(globalId.id, globalId.sectionId, llane->get_id()));
				const Lane* rlane = section->get_outer_neighbour(lane->get_id());
				if (rlane) lane->append_outer_neighbour(GlobalLaneId(globalId.id, globalId.sectionId, rlane->get_id()));
			}
		}
	}
	// get predecessors and successors
	// relations stored in lane
	for (Road* road = front(); road != NULL; road = next(road)) {
		for (LaneSection* section = road->front(); section != NULL; section = road->next(section)) {
			for (Lane* lane = section->front(); lane != NULL; lane = section->next(lane)) {
				GlobalLaneId globalId = lane->get_globalId();
				int preLaneId = globalId.laneId == 0 ? 0 : lane->get_predecessor();
				int sucLaneId = globalId.laneId == 0 ? 0 : lane->get_successor();
				GlobalLaneId preGlobalId;
				GlobalLaneId sucGlobalId;
				if (preLaneId != INT_MAX) {
					if (section != road->front()) {
						preGlobalId = GlobalLaneId(globalId.id, globalId.sectionId - 1, preLaneId);
						(*this)[preGlobalId]->append_successor(globalId);
					}
					else {
						const vector<Link>& preLinks = (*this)[globalId.id]->get_predecessors();
						if (preLinks.size() == 1 && preLinks[0].elementType == "road") {
							if (preLinks[0].contactPoint == "start") {
								preGlobalId = (*(*this)[preLinks[0].elementId]->front())[preLaneId]->get_globalId();
								(*this)[preGlobalId]->append_predecessor(globalId);
							}
							else if (preLinks[0].contactPoint == "end") {
								preGlobalId = (*(*this)[preLinks[0].elementId]->back())[preLaneId]->get_globalId();
								(*this)[preGlobalId]->append_successor(globalId);
							}
						}
					}
					if (preGlobalId.id != "") lane->append_predecessor(preGlobalId);
				}
				if (sucLaneId != INT_MAX) {
					if (road->next(section) != NULL) {
						sucGlobalId = GlobalLaneId(globalId.id, globalId.sectionId + 1, sucLaneId);
						(*this)[sucGlobalId]->append_predecessor(globalId);
					}
					else {
						const vector<Link>& sucLinks = (*this)[globalId.id]->get_successors();
						if (sucLinks.size() == 1 && sucLinks[0].elementType == "road") {
							if (sucLinks[0].contactPoint == "start") {
								sucGlobalId = (*(*this)[sucLinks[0].elementId]->front())[sucLaneId]->get_globalId();
								(*this)[sucGlobalId]->append_predecessor(globalId);
							}
							else if (sucLinks[0].contactPoint == "end") {
								sucGlobalId = (*(*this)[sucLinks[0].elementId]->back())[sucLaneId]->get_globalId();
								(*this)[sucGlobalId]->append_successor(globalId);
							}
						}
					}
					if (sucGlobalId.id != "") lane->append_successor(sucGlobalId);
				}
			}
		}
	}
	// relations stored in junctions
	// unfinished
}

bool RoadMap::append_road(Road& road) {
	// append road
	string id = road.get_id();
	map<string, Road>::iterator iter = _roadmap.find(id);
	if (id == "" || iter != _roadmap.end() || _initialized) {
		if (id == "") cout << "RoadMap: append road error: empty id" << endl;
		if (iter != _roadmap.end()) cout << "RoadMap: append road error: duplicated id" << endl;
		if (_initialized) cout << "RoadMap: append road error: roadmap has been initialized" << endl;
		return false;
	}
	_roadmap[id] = road;
	// append signals
	const vector<Signal>& signals = road.get_signals();
	vector<Signal>::const_iterator signal_iter = signals.begin();
	for ( ; signal_iter != signals.end(); signal_iter++) {
		if (!signal_iter->get_isReference()) {
			string signal_id = signal_iter->get_id();
			map<string, Signal>::const_iterator map_iter = _signalmap.find(id);
			if (map_iter == _signalmap.end()) {
				_signalmap[signal_id] = *signal_iter;
				_signalroad[signal_id] = id;
			}
		}
	}
	return true;
}

bool RoadMap::append_obstacle(Obstacle& obstacle) {
	// append obstacle
	string id = obstacle.getId();
	map<string, Obstacle>::iterator iter = _obstacles.find(id);
	if (id == "" || iter != _obstacles.end() || _initialized) {
		if (id == "") cout << "RoadMap: append obstacle error: empty id" << endl;
		if (iter != _obstacles.end()) cout << "RoadMap: append obstacle error: duplicated id" << endl;
		if (_initialized) cout << "RoadMap: append obstacle error: roadmap has been initialized" << endl;
		return false;
	}
	_obstacles[id] = obstacle;
	return true;
}

void RoadMap::get_signal(const string signalId, const Signal* signal, string &roadId) const {
	map<string, Signal>::const_iterator map_iter = _signalmap.find(signalId);
	if (map_iter == _signalmap.end()) {
		signal = NULL;
	}
	else {
		signal = &(map_iter->second);
		map<string, string>::const_iterator road_iter = _signalroad.find(signalId);
		roadId = road_iter->second;
	}
}

bool RoadMap::append_junction(const Junction &junction) {
	string id = junction.get_id();
	map<string, Junction>::iterator iter = _junctionmap.find(id);
	if (id == "" || iter != _junctionmap.end() || _initialized) {
		if (id == "") cout << "RoadMap: append junction error: empty id" << endl;
		if (iter != _junctionmap.end()) cout << "RoadMap: append junction error: duplicated id" << endl;
		if (_initialized) cout << "RoadMap: append junction error: roadmap has been initialized" << endl;
		return false;
	}
	_junctionmap[id] = junction;
	return true;
}

const Junction* RoadMap::find_junction(const string id) const {
	map<string, Junction>::const_iterator iter = _junctionmap.find(id);
	if (iter == _junctionmap.end()) return NULL;
	else return &(iter->second);
}

const vector<Signal> RoadMap::get_signals() const {
    vector<Signal> res;
    for (map<string, Road>::const_iterator itr = _roadmap.begin();
	     itr != _roadmap.end(); itr++) {
		res.insert(res.end(), itr->second.get_signals().begin(), itr->second.get_signals().end());
	}
	return res;
}

const vector<Signal> RoadMap::get_signals(const string road_id) const {
    vector<Signal> res;
	map<string, Road>::const_iterator _roadptr = _roadmap.find(road_id);
    if (_roadptr == _roadmap.end()) return res;
    return _roadptr->second.get_signals();
}

vector<const Obstacle*> RoadMap::get_obstacles() const {
	vector<const Obstacle*> res;
	for (map<string, Obstacle>::const_iterator it = _obstacles.begin(); it != _obstacles.end(); it++)
		res.push_back(&(it->second));
	return res;
}

RoadMap* RoadMap::_proadmap = NULL;
RoadMap* RoadMap::get_instance(string filename){
	if (!_proadmap) {
		if (filename == "") {
			cout << "error: RoadMap::get_instance: no input filename" << endl;
			return NULL;
		}
		_proadmap = new RoadMap();
		if (!process_doc(*_proadmap, filename)) {
			delete _proadmap;
			_proadmap = NULL;
		}
		else
			_proadmap->init();
	}
	return _proadmap;
}

}
