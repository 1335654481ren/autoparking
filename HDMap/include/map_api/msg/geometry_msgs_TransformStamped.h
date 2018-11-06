#ifndef GEOMETRY_MSGS_TRANSFORMSTAMPED_H
#define GEOMETRY_MSGS_TRANSFORMSTAMPED_H

#include "std_msgs_Header.h"
#include "geometry_msgs_Transform.h"
#include <string>
#include <vector>
using namespace std;

namespace geometry_msgs {

class TransformStamped
{
public:
  TransformStamped() {};
  ~TransformStamped() {};

  std_msgs::Header header;
  string child_frame_id;
  geometry_msgs::Transform transform;
};
} // namespace geometry_msgs

#endif
