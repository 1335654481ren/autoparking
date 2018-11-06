#ifndef AUTODRIVE_MSGS_GLOBALLANEID_H
#define AUTODRIVE_MSGS_GLOBALLANEID_H

#include <string>
#include <vector>
using namespace std;

namespace autodrive_msgs {

class GlobalLaneId
{
public:
  GlobalLaneId() {};
  ~GlobalLaneId() {};

  string id;
  int sectionId;
  int laneId;
};
} // namespace autodrive_msgs

#endif
