#ifndef VISUALIZATION_MSGS_INTERACTIVEMARKERFEEDBACK_H
#define VISUALIZATION_MSGS_INTERACTIVEMARKERFEEDBACK_H

#include "std_msgs_Header.h"
#include "geometry_msgs_Pose.h"
#include "geometry_msgs_Point.h"
#include <string>
#include <vector>
using namespace std;

namespace visualization_msgs {

class InteractiveMarkerFeedback
{
public:
  InteractiveMarkerFeedback() {};
  ~InteractiveMarkerFeedback() {};

  enum {
    KEEP_ALIVE = 0,
    POSE_UPDATE = 1,
    MENU_SELECT = 2,
    BUTTON_CLICK = 3,
    MOUSE_DOWN = 4,
    MOUSE_UP = 5
  };

  std_msgs::Header header;
  string client_id;
  string marker_name;
  string control_name;
  unsigned char event_type;
  geometry_msgs::Pose pose;
  unsigned int menu_entry_id;
  geometry_msgs::Point mouse_point;
  unsigned char mouse_point_valid;
};
} // namespace visualization_msgs

#endif
