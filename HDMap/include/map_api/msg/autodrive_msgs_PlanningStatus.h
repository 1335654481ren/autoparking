#ifndef AUTODRIVE_MSGS_PLANNINGSTATUS_H
#define AUTODRIVE_MSGS_PLANNINGSTATUS_H

#include "std_msgs_Header.h"
#include <string>
#include <vector>
using namespace std;

namespace autodrive_msgs {

class PlanningStatus
{
public:
  PlanningStatus() {};
  ~PlanningStatus() {};

  std_msgs::Header header;
};
} // namespace autodrive_msgs

#endif
