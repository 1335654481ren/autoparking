/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: transformType.h
*   Author  : lubing.han
*   Date    : 2016-11-24
*   Describe:
*
********************************************************/
#ifndef TRANSFORMTYPE_H
#define TRANSFORMTYPE_H

#include "globallaneid.h"
#include "road.h"
#include "geometry.h"
#include <map>
#include <string>
#ifndef DEROS
#include "autodrive_msgs/Map.h"
#include <visualization_msgs/MarkerArray.h>
#include <ros/ros.h>
#else
#include "autodrive_msgs_Map.h"
#include "visualization_msgs_MarkerArray.h"
#include "ros_ros.h"
#endif
using namespace std;

namespace opendrive
{

autodrive_msgs::Point pointToMsg(const Point &point);
autodrive_msgs::GlobalLaneId globalLaneIdToMsg(const GlobalLaneId &globalId);
autodrive_msgs::RoadMark roadMarkToMsg(const RoadMark &roadMark);
autodrive_msgs::Lane laneToMsg(const Lane &lane);
autodrive_msgs::LaneSection laneSectionToMsg(LaneSection &laneScetion);
autodrive_msgs::Road roadToMsg(Road &road);
vector<autodrive_msgs::Point> laneToPointsMsg(const Lane &lane);

visualization_msgs::Marker show_points(const vector<Point> &points, string ns, int id);
visualization_msgs::Marker show_point(const Point &point, string ns, int id, double scale);

}

#endif
