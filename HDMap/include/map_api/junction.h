/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: junction.h
*   Author  : lubing.han
*   Date    : 2016-11-19
*   Describe:
*
********************************************************/
#ifndef JUNCTION_H
#define JUNCTION_H

#include <string>
#include <vector>
using namespace std;

namespace opendrive
{

class LaneLink
{
public:
	LaneLink() {}
	LaneLink(int from, int to): _from(from), _to(to) {}
	~LaneLink() {}
	int get_from() const {return _from;}
	int get_to() const {return _to;}
private:
	int _from;
	int _to;
};

class Connection
{
public:
	Connection(string id, string incomingRoad, string connectingRoad, string contactPoint):
		_id(id), _incomingRoad(incomingRoad), _connectingRoad(connectingRoad), _contactPoint(contactPoint) {};
	~Connection() {};
	string get_id() const {return _id;}
	string get_incomingRoad() const {return _incomingRoad;}
	string get_connectingRoad() const {return _connectingRoad;}
	string get_contactPoint() const {return _contactPoint;}
	const vector<LaneLink>& get_laneLinks() const {return _laneLinks;}
	void append_laneLink(const LaneLink &laneLink) {_laneLinks.push_back(laneLink);}
private:
	string _id;
	string _incomingRoad;
	string _connectingRoad;
	string _contactPoint;
	vector<LaneLink> _laneLinks;
};

class Priority
{
public:
	Priority() {}
	Priority(string high, string low): _high(high), _low(low) {}
	~Priority() {};
	string get_high() const {return _high;}
	string get_low() const {return _low;}
private:
	string _high;
	string _low;
};

class JunctionController
{
public:
	JunctionController() {}
	JunctionController(string id, string type, string sequence):
	_id(id), _type(type), _sequence(sequence) {}
	~JunctionController() {}
	string get_id() const {return _id;}
	string get_type() const {return _type;}
	string get_sequence() const {return _sequence;}
private:
	string _id;
	string _type;
	string _sequence;
};

class Junction
{
public:
	Junction() {}
	Junction(string name, string id): _name(name), _id(id) {} 
	~Junction() {}
	string get_name() const {return _name;}
	string get_id() const {return _id;}
	vector<string> get_connectingRoads() const {
		vector<string> roads;
		for (int i = 0; i < _connections.size(); ++i) {
			roads.push_back(_connections[i].get_connectingRoad());
		}
		return roads;
	}
	void append_connection(const Connection &connection) {_connections.push_back(connection);}
	void append_priority(const Priority &priority) {_priorities.push_back(priority);}
	void append_junctionController(const JunctionController &jc) {_junctionControllers.push_back(jc);}
	const vector<Connection>& get_connections() const {return _connections;}
	const vector<Priority>& get_priorities() const {return _priorities;}
	const vector<JunctionController>& get_junctionControllers() const {return _junctionControllers;}
private:
	string _name;
	string _id;
	vector<Connection> _connections;
	vector<Priority> _priorities;
	vector<JunctionController> _junctionControllers;
};

}

#endif