#ifndef AUTODRIVE_MSGS_OPENDRIVE_OBJECT_SHAPE_CURVE_MSG_H
#define AUTODRIVE_MSGS_OPENDRIVE_OBJECT_SHAPE_CURVE_MSG_H

#include "autodrive_msgs_opendrive_object_shape_point_msg.h"
#include <string>
#include <vector>
using namespace std;

namespace autodrive_msgs {

class opendrive_object_shape_curve_msg
{
public:
  opendrive_object_shape_curve_msg() {};
  ~opendrive_object_shape_curve_msg() {};

  vector<autodrive_msgs::opendrive_object_shape_point_msg> point;
};
} // namespace autodrive_msgs

#endif
