/********************************************************
*   Copyright (C) 2018 All rights reserved.
*   
*   Filename: mapviewer.cpp
*   Author  : lubing.han
*   Date    : 2018-05-17
*   Describe:
*
********************************************************/

#include "map_api.h"
#include "road_fun.h"
#include "mapview.h"
#ifndef DEROS
#include <autodrive_msgs/RoadObjects.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/MarkerArray.h>
#else
#include "autodrive_msgs_RoadObjects.h"
#include "ros_ros.h"
#include "tf_transform_datatypes.h"
#include "visualization_msgs_MarkerArray.h"
#endif
#include <set>
#include <stdlib.h>


using namespace opendrive;
using namespace std;


static string mapfile;

autodrive_msgs::Map getOriginMap() {
  autodrive_msgs::Map map;
  const RoadMap* roadmap = RoadMap::get_instance(mapfile);

  vector<const Obstacle*> mapObs = roadmap->get_obstacles();
  for (int i = 0; i < mapObs.size(); ++i)
    map.obs.push_back(mapObs[i]->toMsg());

  const Lane* lane = roadmap->front3();
  const Road* road_iter = NULL;
  int section_id = 0;
  GlobalLaneId lastId("", 0, 0);
  while (lane) {
    if (true) {
      GlobalLaneId nowId = lane->get_globalId();
      vector<Point> points = lane->get_lane_points();
      
      // connect predecessor
      const vector<GlobalLaneId> &pre_ids = lane->get_predecessors();
      if (pre_ids.size() > 0) {
        const Lane* pre_lane = (*roadmap)[pre_ids[0]];
        const vector<Point> &pre_points = pre_lane->get_lane_points();
        RELATION::RelationType relation = pre_lane->get_relation(nowId);
        if (relation == RELATION::PREDECESSOR)
          points.insert(points.begin(), pre_points.front());
        else if (relation == RELATION::SUCCESSOR)
          points.insert(points.begin(), pre_points.back());
        else
          points.insert(points.begin(), points.front());
      }
      else {
        points.insert(points.begin(), points.front());
      }
      points[0].s = points[1].s;
      points[0].theta = points[1].theta;

      // connect successor
      const vector<GlobalLaneId> &suc_ids = lane->get_successors();
      if (suc_ids.size() > 0) {
        const Lane* suc_lane = (*roadmap)[suc_ids[0]];
        const vector<Point> &suc_points = suc_lane->get_lane_points();
        RELATION::RelationType relation = suc_lane->get_relation(nowId);
        if (relation == RELATION::PREDECESSOR)
          points.insert(points.end(), suc_points.front());
        else if (relation == RELATION::SUCCESSOR)
          points.insert(points.end(), suc_points.back());
        else
          points.insert(points.end(), points.back());
      }
      else {
        points.insert(points.end(), points.back());
      }
      points[points.size() - 1].s = points[points.size() - 2].s;
      points[points.size() - 1].theta = points[points.size() - 2].theta;

      // convert to map_msg
      autodrive_msgs::Lane* laneMsg;
      if (nowId.id != lastId.id) {
        map.roads.push_back(autodrive_msgs::Road());
        autodrive_msgs::Road &roadMsg = map.roads.back();
        const Road* roadOpendrive = (*roadmap)[nowId.id];
        roadMsg.id = nowId.id;
        if ((*roadmap).find_junction(roadOpendrive->get_junction()) != NULL)
          roadMsg.junction = roadOpendrive->get_junction();
        roadMsg.laneSections.push_back(autodrive_msgs::LaneSection());
        autodrive_msgs::LaneSection &laneSectionMsg = roadMsg.laneSections.back();
        laneSectionMsg.lanes.push_back(autodrive_msgs::Lane());
        laneMsg = &laneSectionMsg.lanes.back();
        if (road_iter == NULL)
          road_iter = roadmap->front();
        else
          road_iter = roadmap->next(road_iter);
        section_id = 0;
        laneSectionMsg.sOffset = (*road_iter)[section_id]->get_sOffset();
      }
      else if (nowId.sectionId != lastId.sectionId) {
        autodrive_msgs::Road &roadMsg = map.roads.back();
        roadMsg.laneSections.push_back(autodrive_msgs::LaneSection());
        autodrive_msgs::LaneSection &laneSectionMsg = roadMsg.laneSections.back();
        laneSectionMsg.lanes.push_back(autodrive_msgs::Lane());
        laneMsg = &laneSectionMsg.lanes.back();
        ++section_id;
        laneSectionMsg.sOffset = (*road_iter)[section_id]->get_sOffset();
      }
      else {
        autodrive_msgs::LaneSection &laneSectionMsg = map.roads.back().laneSections.back();
        laneSectionMsg.lanes.push_back(autodrive_msgs::Lane());
        laneMsg = &laneSectionMsg.lanes.back();
      }
      laneMsg->globalid = globalLaneIdToMsg(lane->get_globalId());
      const vector<RoadMark>& roadMarks = lane->get_roadmarks();
        for (int i = 0; i < roadMarks.size(); ++i) {
            laneMsg->roadMarks.push_back(roadMarkToMsg(roadMarks[i]));
            const Road* roadOpendrive = (*roadmap)[nowId.id];
            string junction = roadOpendrive->get_junction();
            if (junction != "-1") laneMsg->roadMarks.back().type = "none";
        }
      for (int i = 0; i < points.size(); ++i)
        laneMsg->points.push_back(pointToMsg(points[i]));
      lastId = nowId;
    }
    lane = roadmap->next(lane);
  }
  map.header.stamp = ros::Time::now();
  map.header.frame_id = "/map";
  return map;
}


autodrive_msgs::RoadObjects getOriginRoadObjects() {
  autodrive_msgs::RoadObjects objects;
  const RoadMap* roadmap = RoadMap::get_instance(mapfile);

  // get markers and signals
  objects.header.stamp = ros::Time::now();
  objects.header.frame_id = "/map";
  const Road* road = roadmap->front();
  while (road) {
    const vector<Signal> signals = road->get_signals();
    for (int i = 0; i < signals.size(); ++i) {
      autodrive_msgs::RoadObject object;
      object.header.stamp = ros::Time::now();
      object.header.frame_id = "/map";
      Signal signal = signals[i];
      string roadId = road->get_id();
      if (signal.get_isReference()) {
        const Signal* psignal;
        roadmap->get_signal(signal.get_id(), psignal, roadId);
        if (!psignal) continue;
        signal = *psignal;
        signal.set_s(signals[i].get_s());
        signal.set_t(signals[i].get_t());
        signal.set_orientation(signals[i].get_orientation());
        signal.set_validities(signals[i].get_validities());
      }
      object.id = signal.get_id();
      object.type = signal.get_type();
      object.subtype = signal.get_subtype();

      Point temp_point;
      temp_point.s = signal.get_s(); temp_point.l = signal.get_t();
      if (temp_point.s > 0) sl2xyz(road, temp_point);
      else {
        temp_point = road->get_refline().front();
        temp_point.x += signal.get_dx();
        temp_point.y += signal.get_dy();
      }
      object.position.x = temp_point.x;
      object.position.y = temp_point.y;
      object.position.z = temp_point.z + signal.get_zOffset();
      object.orientation = tf::createQuaternionMsgFromYaw(temp_point.theta + signal.get_hOffset());
      objects.objs.push_back(object);
    }
    const vector<Marker> markers = road->get_markers();
    for (int i = 0; i < markers.size(); ++i) {
      autodrive_msgs::RoadObject object;
      object.header.stamp = ros::Time::now();
      object.header.frame_id = "/map";
      Marker marker = markers[i];
      object.id = marker.get_id();
      object.type = marker.get_type();
        object.name = marker.get_name();
      
      Point temp_point, ave_point;
      object.orientation.w = 1.0;
      const vector<CornerRoad> cornerRoads = marker.get_outline().get_cornerRoads();
      for (int i = 0; i < cornerRoads.size(); ++i) {
        temp_point.s = cornerRoads[i].get_s(); temp_point.l = cornerRoads[i].get_t();
        sl2xyz(road, temp_point);
        geometry_msgs::Point p;
        p.x = temp_point.x;
        p.y = temp_point.y;
        p.z = temp_point.z + marker.get_zOffset();
          ave_point = ave_point + temp_point;
        object.polygon.push_back(p);
      }
        if (cornerRoads.size() > 0) {
            object.position.x = ave_point.x / cornerRoads.size();
            object.position.y = ave_point.y / cornerRoads.size();
            object.position.z = ave_point.z / cornerRoads.size();
        }
      objects.objs.push_back(object);
    }
    road = roadmap->next(road);
  }
    return objects;
}

#ifdef __cplusplus
extern "C" {
#endif


void setFile(const char* file) {
  mapfile = file;
}

CMap getMap() {
  autodrive_msgs::Map map = getOriginMap();
  CMap cmap;
  cmap.roads_size = map.roads.size();
  cmap.roads = (CRoad*) malloc(sizeof(CRoad) * cmap.roads_size);
  for (int i = 0; i < cmap.roads_size; ++i) {
    autodrive_msgs::Road &road = map.roads[i];
    CRoad &croad = cmap.roads[i];
    croad.id = (char*) malloc(road.id.size() + 1);
    strcpy(croad.id, road.id.c_str());
    croad.laneSections_size = road.laneSections.size();
    croad.laneSections = (CLaneSection*) malloc(sizeof(CLaneSection) * croad.laneSections_size);
    for (int j = 0; j < croad.laneSections_size; ++j) {
      autodrive_msgs::LaneSection &section = road.laneSections[j];
      CLaneSection &csection = croad.laneSections[j];
      csection.lanes_size = section.lanes.size();
      csection.lanes = (CLane*) malloc(sizeof(CLane) * csection.lanes_size);
      for (int k = 0; k < csection.lanes_size; ++k) {
        autodrive_msgs::Lane &lane = section.lanes[k];
        CLane &clane = csection.lanes[k];
          clane.id = lane.globalid.laneId;
        clane.points_size = lane.points.size();
        clane.points = (CPoint*) malloc(sizeof(CPoint) * clane.points_size);
        for (int l = 0; l < clane.points_size; ++l) {
          autodrive_msgs::Point &point = lane.points[l];
          CPoint &cpoint = clane.points[l];
          cpoint.x = point.x;
          cpoint.y = point.y;
          cpoint.z = point.z;
          cpoint.s = point.s;
          cpoint.k = point.k;
          cpoint.theta = point.theta;
          cpoint.type = point.type;
        }
        clane.roadMarks_size = lane.roadMarks.size();
        clane.roadMarks = (CRoadMark*) malloc(sizeof(CRoadMark) * clane.roadMarks_size);
        for (int l = 0; l < clane.roadMarks_size; ++l) {
          autodrive_msgs::RoadMark &roadmark = lane.roadMarks[l];
          CRoadMark &croadmark = clane.roadMarks[l];
          croadmark.sOffset = roadmark.sOffset;
          croadmark.type = (char*) malloc(roadmark.type.size() + 1);
          strcpy(croadmark.type, roadmark.type.c_str());
          croadmark.weight = (char*) malloc(roadmark.weight.size() + 1);
          strcpy(croadmark.weight, roadmark.weight.c_str());
          croadmark.color = (char*) malloc(roadmark.color.size() + 1);
          strcpy(croadmark.color, roadmark.color.c_str());
          croadmark.material = (char*) malloc(roadmark.material.size() + 1);
          strcpy(croadmark.material, roadmark.material.c_str());
          croadmark.width = roadmark.width;
          croadmark.laneChange = (char*) malloc(roadmark.laneChange.size() + 1);
          strcpy(croadmark.laneChange, roadmark.laneChange.c_str());
          croadmark.height = roadmark.height;
        }
      }
      csection.sOffset = section.sOffset;
    }
  }
  return cmap;
}

CRoadObjects getRoadObjects() {
  autodrive_msgs::RoadObjects objects = getOriginRoadObjects();
  CRoadObjects cobjects;
  cobjects.objs_size = objects.objs.size();
  cobjects.objs = (CRoadObject*) malloc(sizeof(CRoadObject) * cobjects.objs_size);
  for (int i = 0; i < cobjects.objs_size; ++i) {
    autodrive_msgs::RoadObject &object = objects.objs[i];
    CRoadObject &cobject = cobjects.objs[i];
    cobject.id = (char*) malloc(object.id.size() + 1);
    strcpy(cobject.id, object.id.c_str());
    cobject.type = (char*) malloc(object.type.size() + 1);
    strcpy(cobject.type, object.type.c_str());
    cobject.subtype = (char*) malloc(object.subtype.size() + 1);
    strcpy(cobject.subtype, object.subtype.c_str());
      cobject.name = (char*) malloc(object.name.size() + 1);
      strcpy(cobject.name, object.name.c_str());
    cobject.height = object.height;
    cobject.position.x = object.position.x;
    cobject.position.y = object.position.y;
    cobject.position.z = object.position.z;
    cobject.orientation.x = object.orientation.x;
    cobject.orientation.y = object.orientation.y;
    cobject.orientation.z = object.orientation.z;
    cobject.orientation.w = object.orientation.w;
    cobject.polygon_size = object.polygon.size();
    cobject.polygon = (CGeoPoint*) malloc(sizeof(CGeoPoint) * cobject.polygon_size);
    for (int i = 0; i < cobject.polygon_size; ++i) {
      geometry_msgs::Point &point = object.polygon[i];
      CGeoPoint &cpoint = cobject.polygon[i];
      cpoint.x = point.x;
      cpoint.y = point.y;
      cpoint.z = point.z;
    }
  }
  return cobjects;
}

CRoadObjects getStations() {
    autodrive_msgs::RoadObjects oldObjects = getOriginRoadObjects();
    autodrive_msgs::RoadObjects objects;
    CRoadObjects cobjects;
    for (int i = 0; i < objects.objs.size(); ++i) {
        if (oldObjects.objs[i].type == "station")
            objects.objs.push_back(oldObjects.objs[i]);
    }
    cobjects.objs_size = objects.objs.size();
    cobjects.objs = (CRoadObject*) malloc(sizeof(CRoadObject) * cobjects.objs_size);
    for (int i = 0; i < cobjects.objs_size; ++i) {
        autodrive_msgs::RoadObject &object = objects.objs[i];
        CRoadObject &cobject = cobjects.objs[i];
        cobject.id = (char*) malloc(object.id.size() + 1);
        strcpy(cobject.id, object.id.c_str());
        cobject.type = (char*) malloc(object.type.size() + 1);
        strcpy(cobject.type, object.type.c_str());
        cobject.subtype = (char*) malloc(object.subtype.size() + 1);
        strcpy(cobject.subtype, object.subtype.c_str());
        cobject.name = (char*) malloc(object.name.size() + 1);
        strcpy(cobject.name, object.name.c_str());
        cobject.height = object.height;
        cobject.position.x = object.position.x;
        cobject.position.y = object.position.y;
        cobject.position.z = object.position.z;
        cobject.orientation.x = object.orientation.x;
        cobject.orientation.y = object.orientation.y;
        cobject.orientation.z = object.orientation.z;
        cobject.orientation.w = object.orientation.w;
        cobject.polygon_size = object.polygon.size();
        cobject.polygon = (CGeoPoint*) malloc(sizeof(CGeoPoint) * cobject.polygon_size);
        for (int i = 0; i < cobject.polygon_size; ++i) {
            geometry_msgs::Point &point = object.polygon[i];
            CGeoPoint &cpoint = cobject.polygon[i];
            cpoint.x = point.x;
            cpoint.y = point.y;
            cpoint.z = point.z;
        }
    }
    return cobjects;
}

#ifdef __cplusplus
}
#endif
