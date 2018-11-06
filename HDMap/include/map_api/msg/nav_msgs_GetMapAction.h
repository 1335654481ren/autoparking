#ifndef NAV_MSGS_GETMAPACTION_H
#define NAV_MSGS_GETMAPACTION_H

#include "nav_msgs_GetMapActionGoal.h"
#include "nav_msgs_GetMapActionResult.h"
#include "nav_msgs_GetMapActionFeedback.h"
#include <string>
#include <vector>
using namespace std;

namespace nav_msgs {

class GetMapAction
{
public:
  GetMapAction() {};
  ~GetMapAction() {};

  nav_msgs::GetMapActionGoal action_goal;
  nav_msgs::GetMapActionResult action_result;
  nav_msgs::GetMapActionFeedback action_feedback;
};
} // namespace nav_msgs

#endif
