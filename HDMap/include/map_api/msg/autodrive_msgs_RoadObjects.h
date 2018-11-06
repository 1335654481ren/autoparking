#ifndef AUTODRIVE_MSGS_ROADOBJECTS_H
#define AUTODRIVE_MSGS_ROADOBJECTS_H

#include "std_msgs_Header.h"
#include "autodrive_msgs_RoadObject.h"
#include <string>
#include <vector>
using namespace std;

namespace autodrive_msgs {

class RoadObjects
{
public:
  RoadObjects() {};
  ~RoadObjects() {};

  std_msgs::Header header;
  vector<autodrive_msgs::RoadObject> objs;
};
} // namespace autodrive_msgs

#endif
