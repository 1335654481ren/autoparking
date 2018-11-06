#ifndef GEOMETRY_MSGS_POSEARRAY_H
#define GEOMETRY_MSGS_POSEARRAY_H

#include "std_msgs_Header.h"
#include "geometry_msgs_Pose.h"
#include <string>
#include <vector>
using namespace std;

namespace geometry_msgs {

class PoseArray
{
public:
  PoseArray() {};
  ~PoseArray() {};

  std_msgs::Header header;
  vector<geometry_msgs::Pose> poses;
};
} // namespace geometry_msgs

#endif
