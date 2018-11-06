/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: lane_converter.h
*   Author  : lubing.han
*   Date    : 2017-03-07
*   Describe:
*
********************************************************/
#ifndef LANE_CONVERTER_H
#define LANE_CONVERTER_H

#include "map_api.h"
#include "envmarkers.h"
#include <vector>
#include <string>
using namespace std;

namespace opendrive {

class MapInfo
{
public:
	MapInfo() {}
	MapInfo(int envId, const GlobalLaneId &laneId, bool re, double es, double ms, double l);
	~MapInfo() {}
	double map2envS(double s) const;
	double env2mapS(double s) const;
	void get_envSLT(Point &p) const;
	void get_mapSLT(Point &p) const;

	int globalEnvId;
	GlobalLaneId globalLaneId;
	bool reversed;
	double envSOffset;
	double mapSOffset;
	double length;
};

class EnvLane
{
public:
	EnvLane() {}
	EnvLane(int globalId, double offset):
		_globalEnvId(globalId), _envSOffset(offset), _length(0.0) {}
	EnvLane(const EnvLane& envLane, double startS, double endS);
	~EnvLane() {}
	int get_globalEnvId() const {return _globalEnvId;}
	string get_type() const {return _type;}
	double get_length() const {return _length;}
	double get_envSOffset() const {return _envSOffset;}
	double get_endEnvS() const {return _length + _envSOffset;}
	vector<pair<char, int> > get_tags() const {return _tags;}
	bool get_nearest_stopLine(double s, pair<double, string>& stopLine) const;
	bool get_nearest_crossWalk(double s, pair<double, double>& ns) const;
	bool get_nearest_junctionS(double s, pair<double, double>& ns) const;
	bool get_parkingSpace(string id, ParkingSpace& parkingSpace) const;
	bool get_parking_pose(string id, geometry_msgs::Pose& pose) const;
	bool get_parking_s(string id, double& s) const;
	
	const vector<StopLine>& get_stopLines() const {return _stopLines;}
	const vector<CrossWalk>& get_crossWalks() const {return _crossWalks;}
	const vector<ParkingSpace>& get_parkingSpaces() const {return _parkingSpaces;}
	const vector<NormalObject>& get_normalObjects() const {return _normalObjects;}
	const vector<pair<double, double> >& get_junctionSs() const {return _junctionSs;}
	// direction: -1:inner, 1:outer
	bool check_laneChange(int direction, double startS, double endS = -1);
	// deprecated
	double get_nearest_junctionS(double curS) const;
	// deprecated
	double get_junctionStartS() const {return _junctionStartS;}
	// deprecated
	double get_junctionEndS() const {return _junctionEndS;}
	const GlobalLaneId* get_globalId(double s) const;
	const vector<GlobalLaneId>& get_globalIds() const {return _globalIds;}
	const MapInfo* get_mapInfo(double s) const;
	const vector<MapInfo>& get_mapInfos() const {return _mapInfos;}

	void set_type(string type) {_type = type;}
	// deprecated
	void set_junctionStartS(double s) {_junctionStartS = s;}
	// deprecated
	void set_junctionEndS(double s) {_junctionEndS = s;}
	void append_stopLine(const StopLine& stopLine) {_stopLines.push_back(stopLine);}
	void append_crossWalk(const CrossWalk& crossWalk) {_crossWalks.push_back(crossWalk);}
	void append_parkingSpace(const ParkingSpace& parkingSpace) {_parkingSpaces.push_back(parkingSpace);}
	void append_normalObject(const NormalObject& normalObject) {_normalObjects.push_back(normalObject);}
	void append_junctionS(const pair<double, double>& junctionS) {_junctionSs.push_back(junctionS);}
	void append_innerLaneChange(const pair<double, bool>& innerLaneChange) {_innerLaneChanges.push_back(innerLaneChange);}
	void append_outerLaneChange(const pair<double, bool>& outerLaneChange) {_outerLaneChanges.push_back(outerLaneChange);}
	void append_tag(pair<char, int> tag) {_tags.push_back(tag);}

	void append_lane(const Lane* lane, bool re, string direction);
	double get_remain(double s) const {return _envSOffset + _length - s;}
	void print() const;
	bool is_Tlane() const;
	bool is_Rlane() const;
	bool is_Tlane(int &id) const;
	bool is_Rlane(int &id) const;
	bool is_Flane() const;
	bool is_Jlane() const;
	bool get_tagindex(char type, int& index) const;
	// deprecated
	vector<Point> get_center_line() const;
	// deprecated
	void get_border_lines(vector<Point> &points1, vector<Point> &points2) const;
	
private:
	int get_index_by_s(double s) const;

	int _globalEnvId;
	string _type;
	double _envSOffset;
	double _length;
	
	vector<pair<char, int> > _tags;
	vector<StopLine> _stopLines;
	vector<CrossWalk> _crossWalks;
	vector<ParkingSpace> _parkingSpaces;
	vector<NormalObject> _normalObjects;
	vector<pair<double, double> > _junctionSs;
	vector<pair<double, bool> > _innerLaneChanges, _outerLaneChanges;
	double _junctionStartS, _junctionEndS;

	vector<GlobalLaneId> _globalIds;
	vector<MapInfo> _mapInfos;
};

}

#endif
