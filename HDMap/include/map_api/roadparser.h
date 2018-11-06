/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: roadparser.h
*   Author  : lubing.han
*   Date    : 2016-11-17
*   Describe: Test open dirve parser
*
********************************************************/
#include "arc.h"
#include "poly3.h"
#include "Spiral.h"
#include "line.h"
#include "tinyxml2.h"
#include "roadmap.h"
#include "junction.h"
#include "LaneSection.h"
#include "paramPoly3.h"

using namespace std;
using namespace tinyxml2;

namespace opendrive
{

bool process_doc(RoadMap &roadmap, string filepath);
bool process_link(Road* road, XMLElement* link);
bool process_planview(Road* road, XMLElement* planview);
bool process_lanes(Road* road, XMLElement* lanes);
bool process_lane(Lane& lane, XMLElement* ele);
bool process_signals(Road* road, XMLElement* signals); 
bool process_markers(Road* road, XMLElement* markers);
bool process_elevations(Road* road, XMLElement* elevations);
Junction process_junction(XMLElement* ele);
Connection process_connection(XMLElement* ele);
Priority process_priority(XMLElement* ele);
JunctionController process_junctionController(XMLElement* ele);

}
