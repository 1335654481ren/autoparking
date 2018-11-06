#ifndef AUTODRIVE_MSGS_OPENDRIVE_OBJECT_SHAPE_POINT_SEGMENT_MSG_H
#define AUTODRIVE_MSGS_OPENDRIVE_OBJECT_SHAPE_POINT_SEGMENT_MSG_H

#include "autodrive_msgs_opendrive_object_shape_point_segment_style_msg.h"
#include "autodrive_msgs_opendrive_object_shape_color_msg.h"
#include "autodrive_msgs_opendrive_object_shape_color_msg.h"
#include "autodrive_msgs_opendrive_object_shape_color_msg.h"
#include <string>
#include <vector>
using namespace std;

namespace autodrive_msgs {

class opendrive_object_shape_point_segment_msg
{
public:
  opendrive_object_shape_point_segment_msg() {};
  ~opendrive_object_shape_point_segment_msg() {};

  autodrive_msgs::opendrive_object_shape_point_segment_style_msg style;
  unsigned char hasColor;
  autodrive_msgs::opendrive_object_shape_color_msg color;
  unsigned char hasLeftColor;
  autodrive_msgs::opendrive_object_shape_color_msg leftColor;
  unsigned char hasRightColor;
  autodrive_msgs::opendrive_object_shape_color_msg rightColor;
};
} // namespace autodrive_msgs

#endif
