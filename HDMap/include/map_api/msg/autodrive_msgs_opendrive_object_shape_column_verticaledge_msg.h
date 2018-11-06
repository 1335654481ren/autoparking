#ifndef AUTODRIVE_MSGS_OPENDRIVE_OBJECT_SHAPE_COLUMN_VERTICALEDGE_MSG_H
#define AUTODRIVE_MSGS_OPENDRIVE_OBJECT_SHAPE_COLUMN_VERTICALEDGE_MSG_H

#include "autodrive_msgs_opendrive_object_shape_color_msg.h"
#include "autodrive_msgs_opendrive_object_shape_color_msg.h"
#include <string>
#include <vector>
using namespace std;

namespace autodrive_msgs {

class opendrive_object_shape_column_verticaledge_msg
{
public:
  opendrive_object_shape_column_verticaledge_msg() {};
  ~opendrive_object_shape_column_verticaledge_msg() {};

  double height;
  unsigned char hasLowerColor;
  autodrive_msgs::opendrive_object_shape_color_msg lowerColor;
  unsigned char hasHigherColor;
  autodrive_msgs::opendrive_object_shape_color_msg higherColor;
};
} // namespace autodrive_msgs

#endif
