#ifndef AUTODRIVE_MSGS_PLANNINGTRAJARRAY_H
#define AUTODRIVE_MSGS_PLANNINGTRAJARRAY_H

#include "std_msgs_Header.h"
#include "autodrive_msgs_PlanningTraj.h"
#include <string>
#include <vector>
using namespace std;

namespace autodrive_msgs {

class PlanningTrajArray
{
public:
  PlanningTrajArray() {};
  ~PlanningTrajArray() {};

  std_msgs::Header header;
  vector<autodrive_msgs::PlanningTraj> trajectories;
};
} // namespace autodrive_msgs

#endif
