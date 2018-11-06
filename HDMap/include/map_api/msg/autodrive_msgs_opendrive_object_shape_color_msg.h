#ifndef AUTODRIVE_MSGS_OPENDRIVE_OBJECT_SHAPE_COLOR_MSG_H
#define AUTODRIVE_MSGS_OPENDRIVE_OBJECT_SHAPE_COLOR_MSG_H

#include <string>
#include <vector>
using namespace std;

namespace autodrive_msgs {

class opendrive_object_shape_color_msg
{
public:
  opendrive_object_shape_color_msg() {};
  ~opendrive_object_shape_color_msg() {};

  unsigned char r;
  unsigned char g;
  unsigned char b;
};
} // namespace autodrive_msgs

#endif
