#ifndef AUTODRIVE_MSGS_LANESECTION_H
#define AUTODRIVE_MSGS_LANESECTION_H

#include "autodrive_msgs_Lane.h"
#include <string>
#include <vector>
using namespace std;

namespace autodrive_msgs {

class LaneSection
{
public:
  LaneSection() {};
  ~LaneSection() {};

  vector<autodrive_msgs::Lane> lanes;
  double sOffset;
};
} // namespace autodrive_msgs

#endif
