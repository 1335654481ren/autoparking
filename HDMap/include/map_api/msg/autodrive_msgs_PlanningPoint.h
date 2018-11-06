#ifndef AUTODRIVE_MSGS_PLANNINGPOINT_H
#define AUTODRIVE_MSGS_PLANNINGPOINT_H

#include <string>
#include <vector>
using namespace std;

namespace autodrive_msgs {

class PlanningPoint
{
public:
  PlanningPoint() {};
  ~PlanningPoint() {};

  double x;
  double y;
  double z;
  double s;
  double l;
  double k;
  double theta;
  double speed;
  short i;
  int gear;
};
} // namespace autodrive_msgs

#endif
