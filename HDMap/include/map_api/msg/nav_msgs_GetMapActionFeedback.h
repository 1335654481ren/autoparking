#ifndef NAV_MSGS_GETMAPACTIONFEEDBACK_H
#define NAV_MSGS_GETMAPACTIONFEEDBACK_H

#include "std_msgs_Header.h"
#include "actionlib_msgs_GoalStatus.h"
#include "nav_msgs_GetMapFeedback.h"
#include <string>
#include <vector>
using namespace std;

namespace nav_msgs {

class GetMapActionFeedback
{
public:
  GetMapActionFeedback() {};
  ~GetMapActionFeedback() {};

  std_msgs::Header header;
  actionlib_msgs::GoalStatus status;
  nav_msgs::GetMapFeedback feedback;
};
} // namespace nav_msgs

#endif
