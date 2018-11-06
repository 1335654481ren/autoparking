#ifndef NAV_MSGS_GETMAPACTIONRESULT_H
#define NAV_MSGS_GETMAPACTIONRESULT_H

#include "std_msgs_Header.h"
#include "actionlib_msgs_GoalStatus.h"
#include "nav_msgs_GetMapResult.h"
#include <string>
#include <vector>
using namespace std;

namespace nav_msgs {

class GetMapActionResult
{
public:
  GetMapActionResult() {};
  ~GetMapActionResult() {};

  std_msgs::Header header;
  actionlib_msgs::GoalStatus status;
  nav_msgs::GetMapResult result;
};
} // namespace nav_msgs

#endif
