#ifndef AUTODRIVE_MSGS_OPENDRIVE_OBJECT_SHAPE_POLYGON_MSG_H
#define AUTODRIVE_MSGS_OPENDRIVE_OBJECT_SHAPE_POLYGON_MSG_H

#include "autodrive_msgs_opendrive_object_shape_point_msg.h"
#include <string>
#include <vector>
using namespace std;

namespace autodrive_msgs {

class opendrive_object_shape_polygon_msg
{
public:
  opendrive_object_shape_polygon_msg() {};
  ~opendrive_object_shape_polygon_msg() {};

  unsigned char nonconvex;
  vector<autodrive_msgs::opendrive_object_shape_point_msg> point;
};
} // namespace autodrive_msgs

#endif
