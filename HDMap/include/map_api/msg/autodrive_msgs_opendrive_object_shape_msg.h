#ifndef AUTODRIVE_MSGS_OPENDRIVE_OBJECT_SHAPE_MSG_H
#define AUTODRIVE_MSGS_OPENDRIVE_OBJECT_SHAPE_MSG_H

#include "autodrive_msgs_opendrive_object_shape_curve_msg.h"
#include "autodrive_msgs_opendrive_object_shape_polygon_msg.h"
#include "autodrive_msgs_opendrive_object_shape_column_msg.h"
#include <string>
#include <vector>
using namespace std;

namespace autodrive_msgs {

class opendrive_object_shape_msg
{
public:
  opendrive_object_shape_msg() {};
  ~opendrive_object_shape_msg() {};

  unsigned char hasCurve;
  autodrive_msgs::opendrive_object_shape_curve_msg curve;
  unsigned char hasPolygon;
  autodrive_msgs::opendrive_object_shape_polygon_msg polygon;
  unsigned char hasColumn;
  autodrive_msgs::opendrive_object_shape_column_msg column;
};
} // namespace autodrive_msgs

#endif
