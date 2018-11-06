#ifndef AUTODRIVE_MSGS_OBSTACLE_H
#define AUTODRIVE_MSGS_OBSTACLE_H

#include "std_msgs_Header.h"
#include "geometry_msgs_Point.h"
#include "geometry_msgs_Vector3.h"
#include "geometry_msgs_Point.h"
#include "autodrive_msgs_PlanningTraj.h"
#include "geometry_msgs_Quaternion.h"
#include <string>
#include <vector>
using namespace std;

namespace autodrive_msgs {

class Obstacle
{
public:
  Obstacle() {};
  ~Obstacle() {};

  std_msgs::Header header;
  int ObsId;
  geometry_msgs::Point ObsPosition;
  float ObsTheta;
  geometry_msgs::Vector3 Velocity;
  float Length;
  float Width;
  float Height;
  vector<geometry_msgs::Point> PolygonPoints;
  autodrive_msgs::PlanningTraj predictionTraj;
  float Life;
  short Classification;
  short CUBE;
  string ResourceFile;
  geometry_msgs::Quaternion Orientation;
};
} // namespace autodrive_msgs

#endif
