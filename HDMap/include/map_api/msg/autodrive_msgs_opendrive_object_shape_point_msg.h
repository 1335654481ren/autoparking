#ifndef AUTODRIVE_MSGS_OPENDRIVE_OBJECT_SHAPE_POINT_MSG_H
#define AUTODRIVE_MSGS_OPENDRIVE_OBJECT_SHAPE_POINT_MSG_H

#include "autodrive_msgs_opendrive_object_shape_point_segment_msg.h"
#include <string>
#include <vector>
using namespace std;

namespace autodrive_msgs {

class opendrive_object_shape_point_msg
{
public:
  opendrive_object_shape_point_msg() {};
  ~opendrive_object_shape_point_msg() {};

  double x;
  double y;
  double z;
  unsigned char exact;
  unsigned char hasSegment;
  autodrive_msgs::opendrive_object_shape_point_segment_msg segment;
};
} // namespace autodrive_msgs

#endif
