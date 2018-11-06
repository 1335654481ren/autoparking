#ifndef AUTODRIVE_MSGS_ROADOBJECT_H
#define AUTODRIVE_MSGS_ROADOBJECT_H

#include "std_msgs_Header.h"
#include "geometry_msgs_Point.h"
#include "geometry_msgs_Quaternion.h"
#include "geometry_msgs_Point.h"
#include <string>
#include <vector>
using namespace std;

namespace autodrive_msgs {

class RoadObject
{
public:
  RoadObject() {};
  ~RoadObject() {};

  std_msgs::Header header;
  string id;
  string type;
  string subtype;
    string name;
  float height;
  geometry_msgs::Point position;
  geometry_msgs::Quaternion orientation;
  vector<geometry_msgs::Point> polygon;
};
} // namespace autodrive_msgs

#endif
