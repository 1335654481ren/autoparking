/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: globallaneid.h
*   Author  : lubing.han
*   Date    : 2016-11-21
*   Describe:
*
********************************************************/
#ifndef GLOBALLANEID_H
#define GLOBALLANEID_H

// #include <vector>
#include <string>
#include <stdio.h>
using namespace std;

namespace opendrive
{


class GlobalLaneId
{
public:
	GlobalLaneId() {id = "";};
	GlobalLaneId(string i, int s, int l): id(i), sectionId(s), laneId(l) {}
	~GlobalLaneId() {};
	bool operator<(GlobalLaneId const &b) const;
	bool operator==(GlobalLaneId const &b) const;
	bool operator!=(GlobalLaneId const &b) const;
	void print() const;
  string get_str();
	// bool append_predecessor(const GlobalLaneId &id);
	// bool append_successor(const GlobalLaneId &id);
	string id;
	int sectionId;
	int laneId;
	// vector<const GlobalLaneId*> predecessor;
	// vector<const GlobalLaneId*> successor;
};

}

#endif
