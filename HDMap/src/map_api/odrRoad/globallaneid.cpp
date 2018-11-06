/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: globallaneid.cpp
*   Author  : lubing.han
*   Date    : 2016-11-21
*   Describe:
*
********************************************************/

#include "globallaneid.h"
#include <iostream>
using namespace std;

namespace opendrive
{

bool GlobalLaneId::operator<(GlobalLaneId const &b) const {
	if (id < b.id)
		return true;
	if (b.id < id)
		return false;
	if (sectionId < b.sectionId)
		return true;
	if (b.sectionId < sectionId)
		return false;
	if (laneId < b.laneId)
		return true;
	return false;
}

bool GlobalLaneId::operator==(GlobalLaneId const &b) const {
	return (id == b.id) && (sectionId == b.sectionId) && (laneId == b.laneId);
}

bool GlobalLaneId::operator!=(GlobalLaneId const &b) const {
	return (id != b.id) || (sectionId != b.sectionId) || (laneId != b.laneId);
}

void GlobalLaneId::print() const {
	cout << "(" << id << ", " << sectionId << ", " << laneId << ")" << endl;
}

string GlobalLaneId::get_str() {
  char res[100];
  sprintf(res, "(%s,%d,%d)", id.c_str(), sectionId, laneId);
  return string(res);
}

// bool GlobalLaneId::append_predecessor(const GlobalLaneId &id) {
// 	for (int i = 0; i < predecessor.size(); ++i) {
// 		if (*(predecessor[i]) == id)
// 			return false;
// 	}
// 	predecessor.push_back(&id);
// 	return true;
// }

// bool GlobalLaneId::append_successor(const GlobalLaneId &id) {
// 	for (int i = 0; i < successor.size(); ++i) {
// 		if (*(successor[i]) == id)
// 			return false;
// 	}
// 	successor.push_back(&id);
// 	return true;
// }

}
