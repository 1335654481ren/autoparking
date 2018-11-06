#ifndef NAV_MSGS_GETMAPACTIONGOAL_H
#define NAV_MSGS_GETMAPACTIONGOAL_H

#include "std_msgs_Header.h"
#include "actionlib_msgs_GoalID.h"
#include "nav_msgs_GetMapGoal.h"
#include <string>
#include <vector>
using namespace std;

namespace nav_msgs {

class GetMapActionGoal
{
public:
  GetMapActionGoal() {};
  ~GetMapActionGoal() {};

  std_msgs::Header header;
  actionlib_msgs::GoalID goal_id;
  nav_msgs::GetMapGoal goal;
};
} // namespace nav_msgs

#endif
