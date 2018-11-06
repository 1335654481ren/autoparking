#ifndef GEOMETRY_MSGS_POSE_H
#define GEOMETRY_MSGS_POSE_H

#include "geometry_msgs_Point.h"
#include "geometry_msgs_Quaternion.h"
#include <string>
#include <vector>
using namespace std;

namespace geometry_msgs {

class Pose
{
public:
  Pose() {};
  ~Pose() {};

  geometry_msgs::Point position;
  geometry_msgs::Quaternion orientation;
};
} // namespace geometry_msgs

#endif
