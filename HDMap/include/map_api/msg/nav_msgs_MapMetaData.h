#ifndef NAV_MSGS_MAPMETADATA_H
#define NAV_MSGS_MAPMETADATA_H

#include "geometry_msgs_Pose.h"
#include <string>
#include <vector>
using namespace std;

namespace nav_msgs {

class MapMetaData
{
public:
  MapMetaData() {};
  ~MapMetaData() {};

  double map_load_time;
  float resolution;
  unsigned int width;
  unsigned int height;
  geometry_msgs::Pose origin;
};
} // namespace nav_msgs

#endif
