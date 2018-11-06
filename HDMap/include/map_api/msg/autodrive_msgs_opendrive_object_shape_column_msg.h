#ifndef AUTODRIVE_MSGS_OPENDRIVE_OBJECT_SHAPE_COLUMN_MSG_H
#define AUTODRIVE_MSGS_OPENDRIVE_OBJECT_SHAPE_COLUMN_MSG_H

#include "autodrive_msgs_opendrive_object_shape_curve_msg.h"
#include "autodrive_msgs_opendrive_object_shape_polygon_msg.h"
#include "autodrive_msgs_opendrive_object_shape_column_verticaledge_msg.h"
#include <string>
#include <vector>
using namespace std;

namespace autodrive_msgs {

class opendrive_object_shape_column_msg
{
public:
  opendrive_object_shape_column_msg() {};
  ~opendrive_object_shape_column_msg() {};

  double height;
  unsigned char hasCurve;
  autodrive_msgs::opendrive_object_shape_curve_msg curve;
  unsigned char hasPolygon;
  autodrive_msgs::opendrive_object_shape_polygon_msg polygon;
  unsigned char hasVerticalEdge;
  autodrive_msgs::opendrive_object_shape_column_verticaledge_msg verticalEdge;
};
} // namespace autodrive_msgs

#endif
