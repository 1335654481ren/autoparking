/********************************************************
*   Copyright (C) 2016 All rights reserved.
*   
*   Filename: envlane_fun.cpp
*   Author  : lubing.han
*   Date    : 2017-07-06
*   Describe:
*
********************************************************/

#include "envlane_fun.h"
#include "grid_filler.h"
#include <algorithm>
namespace opendrive{

void get_center_line(const RoadMap* roadmap, const EnvLane* envLane, vector<Point> &points) {
	points.clear();
	const vector<GlobalLaneId> &globalIds = envLane->get_globalIds();
	const vector<MapInfo> &mapInfos = envLane->get_mapInfos();
	for (int i = 0; i < globalIds.size(); ++i) {
		vector<Point> temp_points = (*roadmap)[globalIds[i]]->get_center();
		for (int j = 0; j < temp_points.size(); ++j)
			mapInfos[i].get_envSLT(temp_points[j]);
		if (mapInfos[i].reversed) reverse(temp_points.begin(), temp_points.end());
		points.insert(points.end(), temp_points.begin(), temp_points.end());
	}
}

void get_border_lines(const RoadMap* roadmap, const EnvLane* envLane,
	vector<Point> &points1, vector<Point> &points2) {
	points1.clear(), points2.clear();
	const vector<GlobalLaneId> &globalIds = envLane->get_globalIds();
	const vector<MapInfo> &mapInfos = envLane->get_mapInfos();
	for (int i = 0; i < globalIds.size(); ++i) {
		vector<Point> temp_points1 = (*roadmap)[globalIds[i]]->get_inner_points();
		vector<Point> temp_points2 = (*roadmap)[globalIds[i]]->get_lane_points();
		for (int j = 0; j < temp_points1.size() && j < temp_points2.size(); ++j) {
			mapInfos[i].get_envSLT(temp_points1[j]);
			mapInfos[i].get_envSLT(temp_points2[j]);
		}
		if (mapInfos[i].reversed) {
			reverse(temp_points1.begin(), temp_points1.end());
			reverse(temp_points2.begin(), temp_points2.end());
		}
		points1.insert(points1.end(), temp_points1.begin(), temp_points1.end());
		points2.insert(points2.end(), temp_points2.begin(), temp_points2.end());
	}
}

vector<GlobalLaneId> get_local_lanes(const map<int, EnvLane>& envLanes, double minS, double maxS) {
	vector<GlobalLaneId> ids;
	map<int, EnvLane>::const_iterator i = envLanes.begin();
	for ( ; i != envLanes.end(); ++i) {
		if (!(i->second.is_Tlane() || i->second.is_Rlane())) continue;
		const vector<GlobalLaneId>& globalIds = i->second.get_globalIds();
		const vector<MapInfo>& mapInfos = i->second.get_mapInfos();
		for (int j = 0; j < globalIds.size(); ++j) {
			if (mapInfos[j].envSOffset < maxS && mapInfos[j].envSOffset + mapInfos[j].length > minS) {
				ids.push_back(globalIds[j]);
			}
		}
	}
	return ids;
}

void fillLaneGrid(const EnvLane& envLane, nav_msgs::OccupancyGrid& grid, double (*eval)(double l)) {
	vector<opendrive::Point> points1, points2;
	envLane.get_border_lines(points1, points2);
	for (int i = 0; i + 1 < points1.size(); ++i) {
		geometry_msgs::Point A, B, C, D;
		A.x = points1[i].x, A.y = points1[i].y;
		B.x = points2[i].x, B.y = points2[i].y;
		C.x = points1[i + 1].x, C.y = points1[i + 1].y;
		D.x = points2[i + 1].x, D.y = points2[i + 1].y;
		vector<geometry_msgs::Point> points;
		points.push_back(A), points.push_back(B), points.push_back(C), points.push_back(D);
		fillLaneSegment(grid, points, eval, COVER_IF_SMALLER);
	}
}

void fillLaneGrid(const EnvLane& envLane, planning_matrix::Grid& grid, double (*eval)(double l)) {
	vector<opendrive::Point> points1, points2;
	envLane.get_border_lines(points1, points2);
	for (int i = 0; i + 1 < points1.size(); ++i) {
		geometry_msgs::Point A, B, C, D;
		A.x = points1[i].x, A.y = points1[i].y;
		B.x = points2[i].x, B.y = points2[i].y;
		C.x = points1[i + 1].x, C.y = points1[i + 1].y;
		D.x = points2[i + 1].x, D.y = points2[i + 1].y;
		vector<geometry_msgs::Point> points;
		points.push_back(A), points.push_back(B), points.push_back(C), points.push_back(D);
		fillLaneSegment(grid, points, eval, COVER_IF_SMALLER);
	}
}

void fillMapGrid(const map<int, EnvLane>& envLanes, const Point& carPoint, nav_msgs::OccupancyGrid& grid, string parkingId) {
	fill(grid.data.begin(), grid.data.end(), 100);
	grid.header.frame_id = "/map";
	grid.header.stamp = ros::Time::now();

	double x = carPoint.x;
	double y = carPoint.y;
	double dx = GRID_WIDTH * GRID_RESOLUTION / 2;
	double dy = GRID_HEIGHT * GRID_RESOLUTION / 2;
	double yaw = 0.0;
	grid.info.origin.orientation = tf::createQuaternionMsgFromYaw(yaw);
	grid.info.origin.position.x = x - dx * cos(yaw) + dy * sin(yaw);
	grid.info.origin.position.y = y - dy * cos(yaw) - dx * sin(yaw);
	grid.info.origin.position.z = 0.5;

	map<int, EnvLane>::const_iterator iter = envLanes.begin();
	for ( ; iter != envLanes.end(); ++iter) {
		int value = 100, index;
		if (iter->second.get_tagindex('T', index) && index == 0) value = 0; // self lane
		else if (iter->second.is_Tlane() && iter->second.get_type() == "driving") value = 0; // same direction lanes
		else value = 50;

		// draw lane
		vector<opendrive::Point> points1, points2;
		iter->second.get_border_lines(points1, points2);
		for (int i = 0; i + 1 < points1.size(); ++i) {
			geometry_msgs::Point A, B, C, D;
			A.x = points1[i].x, A.y = points1[i].y;
			B.x = points2[i].x, B.y = points2[i].y;
			C.x = points1[i + 1].x, C.y = points1[i + 1].y;
			D.x = points2[i + 1].x, D.y = points2[i + 1].y;
			vector<geometry_msgs::Point> points;
			points.push_back(A), points.push_back(B), points.push_back(C), points.push_back(D);
			fillTriangle(grid, value, A, B, C, COVER_IF_SMALLER);
			fillTriangle(grid, value, D, B, C, COVER_IF_SMALLER);
			fillTriangle(grid, value, A, B, D, COVER_IF_SMALLER);
			fillTriangle(grid, value, A, D, C, COVER_IF_SMALLER);
//			fillLaneSegment(grid, points, &simplePiecewiseLinear, COVER_IF_SMALLER);
		}

		// if (!(iter->second.get_tagindex('T', index) && index == 0) && iter->second.get_tags().size() > 0) continue;
		// draw parkingSpaces
//		const vector<ParkingSpace>& parkingSpaces = iter->second.get_parkingSpace(parkingId, );
		ParkingSpace tarParkingSpace;
		bool validParkingId = iter->second.get_parkingSpace(parkingId, tarParkingSpace);
//		for (int i = 0; i < parkingSpaces.size(); ++i) {
		if(validParkingId) {
			const vector<opendrive::Point>& points = tarParkingSpace.get_cornerPoints();
			geometry_msgs::Point A, B, C;
			A.x = points[0].x, A.y = points[0].y;
			B.x = points[1].x, B.y = points[1].y;
			C.x = points[2].x, C.y = points[2].y;
			fillTriangle(grid, 0, A, B, C);
			B = C, C.x = points[3].x, C.y = points[3].y;
			fillTriangle(grid, 0, A, B, C);
		}
		// draw normalObjects
		const vector<NormalObject>& normalObjects = iter->second.get_normalObjects();
		for (int i = 0; i < normalObjects.size(); ++i) {
			const vector<opendrive::Point>& points = normalObjects[i].get_cornerPoints();
			vector<geometry_msgs::Point> geoPoints(points.size());
			for (int j = 0; j < points.size(); ++j)
				geoPoints[j].x = points[j].x, geoPoints[j].y = points[j].y;
			fillPolygon(grid, 100, geoPoints);
		}
	}
	if (ADD_OBS_EDGE) {
		for (int i = 0; i < grid.info.width; ++i) grid.data[i] = 100;
		for (int i = grid.info.width * (grid.info.height - 1); i < grid.info.width * grid.info.height; ++i) grid.data[i] = 100;
		for (int i = 0; i < grid.info.width * grid.info.height; i += grid.info.width) grid.data[i] = 100;
		for (int i = grid.info.width - 1; i < grid.info.width * grid.info.height; i += grid.info.width) grid.data[i] = 100;
	}
}

}
