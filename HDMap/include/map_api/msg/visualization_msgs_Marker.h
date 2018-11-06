#ifndef VISUALIZATION_MSGS_MARKER_H
#define VISUALIZATION_MSGS_MARKER_H

#include "std_msgs_Header.h"
#include "geometry_msgs_Pose.h"
#include "geometry_msgs_Vector3.h"
#include "std_msgs_ColorRGBA.h"
#include "geometry_msgs_Point.h"
#include "std_msgs_ColorRGBA.h"
#include <string>
#include <vector>
using namespace std;

namespace visualization_msgs {

class Marker
{
public:
  Marker() {};
  ~Marker() {};

  enum {
    ARROW = 0,
    CUBE = 1,
    SPHERE = 2,
    CYLINDER = 3,
    LINE_STRIP = 4,
    LINE_LIST = 5,
    CUBE_LIST = 6,
    SPHERE_LIST = 7,
    POINTS = 8,
    TEXT_VIEW_FACING = 9,
    MESH_RESOURCE = 10,
    TRIANGLE_LIST = 11,
    ADD = 0,
    MODIFY = 0,
    DELETE = 2,
    DELETEALL = 3
  };

  std_msgs::Header header;
  string ns;
  int id;
  int type;
  int action;
  geometry_msgs::Pose pose;
  geometry_msgs::Vector3 scale;
  std_msgs::ColorRGBA color;
  double lifetime;
  unsigned char frame_locked;
  vector<geometry_msgs::Point> points;
  vector<std_msgs::ColorRGBA> colors;
  string text;
  string mesh_resource;
  unsigned char mesh_use_embedded_materials;
};
} // namespace visualization_msgs

#endif
