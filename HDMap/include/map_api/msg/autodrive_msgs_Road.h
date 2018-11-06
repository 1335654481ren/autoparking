#ifndef AUTODRIVE_MSGS_ROAD_H
#define AUTODRIVE_MSGS_ROAD_H

#include "autodrive_msgs_LaneSection.h"
#include <string>
#include <vector>
using namespace std;

namespace autodrive_msgs {

class Road
{
public:
  Road() {};
  ~Road() {};

  vector<autodrive_msgs::LaneSection> laneSections;
  string id;
  string junction;
  vector<string> predecessors;
  vector<string> successors;
  vector<string> contactPointPre;
  vector<string> contactPointSuc;
};
} // namespace autodrive_msgs

#endif
