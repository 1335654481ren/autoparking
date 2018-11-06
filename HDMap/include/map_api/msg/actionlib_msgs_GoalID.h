#ifndef ACTIONLIB_MSGS_GOALID_H
#define ACTIONLIB_MSGS_GOALID_H

#include <string>
#include <vector>
using namespace std;

namespace actionlib_msgs {

class GoalID
{
public:
  GoalID() {};
  ~GoalID() {};

  double stamp;
  string id;
};
} // namespace actionlib_msgs

#endif
