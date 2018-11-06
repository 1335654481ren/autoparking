#ifndef AUTODRIVE_MSGS_PLANNINGTRAJ_H
#define AUTODRIVE_MSGS_PLANNINGTRAJ_H

#include "std_msgs_Header.h"
#include "autodrive_msgs_PlanningPoint.h"
#include <string>
#include <vector>
using namespace std;

namespace autodrive_msgs {

class PlanningTraj
{
public:
  PlanningTraj() {};
  ~PlanningTraj() {};

  enum {
    TO_CURRENT_LANE = 0,
    TO_LEFT_LANE = 1,
    TO_RIGHT_LANE = 2,
    SOFT_STOP = 3,
    EMERGENCY_STOP = 4
  };

  std_msgs::Header header;
  vector<autodrive_msgs::PlanningPoint> trajectory;
  unsigned char emergency;
  int decision;
  unsigned char direction;
  unsigned char new_trajectory;
  int type;
};
} // namespace autodrive_msgs

#endif
