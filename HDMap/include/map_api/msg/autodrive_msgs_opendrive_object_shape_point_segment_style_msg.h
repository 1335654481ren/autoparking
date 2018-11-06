#ifndef AUTODRIVE_MSGS_OPENDRIVE_OBJECT_SHAPE_POINT_SEGMENT_STYLE_MSG_H
#define AUTODRIVE_MSGS_OPENDRIVE_OBJECT_SHAPE_POINT_SEGMENT_STYLE_MSG_H

#include <string>
#include <vector>
using namespace std;

namespace autodrive_msgs {

class opendrive_object_shape_point_segment_style_msg
{
public:
  opendrive_object_shape_point_segment_style_msg() {};
  ~opendrive_object_shape_point_segment_style_msg() {};

  string type;
  double width;
};
} // namespace autodrive_msgs

#endif
