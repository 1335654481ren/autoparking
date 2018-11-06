#ifndef ACTIONLIB_MSGS_GOALSTATUSARRAY_H
#define ACTIONLIB_MSGS_GOALSTATUSARRAY_H

#include "std_msgs_Header.h"
#include "actionlib_msgs_GoalStatus.h"
#include <string>
#include <vector>
using namespace std;

namespace actionlib_msgs {

class GoalStatusArray
{
public:
  GoalStatusArray() {};
  ~GoalStatusArray() {};

  std_msgs::Header header;
  vector<actionlib_msgs::GoalStatus> status_list;
};
} // namespace actionlib_msgs

#endif
