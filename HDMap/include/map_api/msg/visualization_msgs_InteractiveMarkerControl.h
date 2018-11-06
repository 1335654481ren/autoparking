#ifndef VISUALIZATION_MSGS_INTERACTIVEMARKERCONTROL_H
#define VISUALIZATION_MSGS_INTERACTIVEMARKERCONTROL_H

#include "geometry_msgs_Quaternion.h"
#include "visualization_msgs_Marker.h"
#include <string>
#include <vector>
using namespace std;

namespace visualization_msgs {

class InteractiveMarkerControl
{
public:
  InteractiveMarkerControl() {};
  ~InteractiveMarkerControl() {};

  enum {
    INHERIT = 0,
    FIXED = 1,
    VIEW_FACING = 2,
    NONE = 0,
    MENU = 1,
    BUTTON = 2,
    MOVE_AXIS = 3,
    MOVE_PLANE = 4,
    ROTATE_AXIS = 5,
    MOVE_ROTATE = 6,
    MOVE_3D = 7,
    ROTATE_3D = 8,
    MOVE_ROTATE_3D = 9
  };

  string name;
  geometry_msgs::Quaternion orientation;
  unsigned char orientation_mode;
  unsigned char interaction_mode;
  unsigned char always_visible;
  vector<visualization_msgs::Marker> markers;
  unsigned char independent_marker_orientation;
  string description;
};
} // namespace visualization_msgs

#endif
