#ifndef VISUALIZATION_MSGS_INTERACTIVEMARKERPOSE_H
#define VISUALIZATION_MSGS_INTERACTIVEMARKERPOSE_H

#include "std_msgs_Header.h"
#include "geometry_msgs_Pose.h"
#include <string>
#include <vector>
using namespace std;

namespace visualization_msgs {

class InteractiveMarkerPose
{
public:
  InteractiveMarkerPose() {};
  ~InteractiveMarkerPose() {};

  std_msgs::Header header;
  geometry_msgs::Pose pose;
  string name;
};
} // namespace visualization_msgs

#endif
