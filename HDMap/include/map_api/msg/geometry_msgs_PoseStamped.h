#ifndef GEOMETRY_MSGS_POSESTAMPED_H
#define GEOMETRY_MSGS_POSESTAMPED_H

#include "std_msgs_Header.h"
#include "geometry_msgs_Pose.h"
#include <string>
#include <vector>
using namespace std;

namespace geometry_msgs {

class PoseStamped
{
public:
  PoseStamped() {};
  ~PoseStamped() {};

  std_msgs::Header header;
  geometry_msgs::Pose pose;
};
} // namespace geometry_msgs

#endif
