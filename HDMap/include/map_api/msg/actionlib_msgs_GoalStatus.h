#ifndef ACTIONLIB_MSGS_GOALSTATUS_H
#define ACTIONLIB_MSGS_GOALSTATUS_H

#include "actionlib_msgs_GoalID.h"
#include <string>
#include <vector>
using namespace std;

namespace actionlib_msgs {

class GoalStatus
{
public:
  GoalStatus() {};
  ~GoalStatus() {};

  enum {
    PENDING = 0,
    ACTIVE = 1,
    PREEMPTED = 2,
    SUCCEEDED = 3,
    ABORTED = 4,
    REJECTED = 5,
    PREEMPTING = 6,
    RECALLING = 7,
    RECALLED = 8,
    LOST = 9
  };

  actionlib_msgs::GoalID goal_id;
  unsigned char status;
  string text;
};
} // namespace actionlib_msgs

#endif
