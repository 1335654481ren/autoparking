#ifndef VISUALIZATION_MSGS_INTERACTIVEMARKER_H
#define VISUALIZATION_MSGS_INTERACTIVEMARKER_H

#include "std_msgs_Header.h"
#include "geometry_msgs_Pose.h"
#include "visualization_msgs_MenuEntry.h"
#include "visualization_msgs_InteractiveMarkerControl.h"
#include <string>
#include <vector>
using namespace std;

namespace visualization_msgs {

class InteractiveMarker
{
public:
  InteractiveMarker() {};
  ~InteractiveMarker() {};

  std_msgs::Header header;
  geometry_msgs::Pose pose;
  string name;
  string description;
  float scale;
  vector<visualization_msgs::MenuEntry> menu_entries;
  vector<visualization_msgs::InteractiveMarkerControl> controls;
};
} // namespace visualization_msgs

#endif
