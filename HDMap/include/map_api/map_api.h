/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: map_api.h
*   Author  : lubing.han
*   Date    : 2016-12-07
*   Describe: Test open dirve parser
*
********************************************************/

#ifndef MAP_API
#define MAP_API

#include "roadmap.h"
#include "roadparser.h"
#include "map_api_parameters.h"
#include "transformType.h"
#ifndef DEROS
#include "autodrive_msgs/Map.h"
#else
#include "autodrive_msgs_Map.h"
#endif
using namespace std;

class MapAPI
{
public:
	MapAPI();
	~MapAPI();
private:
	// opendrive::RoadMap _roadmap;
};

#endif
