#ifndef AUTODRIVE_MSGS_ROADMARK_H
#define AUTODRIVE_MSGS_ROADMARK_H

#include <string>
#include <vector>
using namespace std;

namespace autodrive_msgs {

class RoadMark
{
public:
  RoadMark() {};
  ~RoadMark() {};

  double sOffset;
  string type;
  string weight;
  string color;
  string material;
  double width;
  string laneChange;
  double height;
};
} // namespace autodrive_msgs

#endif
