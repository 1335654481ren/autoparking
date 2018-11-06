#ifndef AUTODRIVE_MSGS_MAP_H
#define AUTODRIVE_MSGS_MAP_H

#include "std_msgs_Header.h"
#include "autodrive_msgs_Road.h"
#include "autodrive_msgs_opendrive_object_msg.h"
#include <string>
#include <vector>
using namespace std;

namespace autodrive_msgs {

class Map
{
public:
  Map() {};
  ~Map() {};

  std_msgs::Header header;
  vector<autodrive_msgs::Road> roads;
  vector<autodrive_msgs::opendrive_object_msg> obs;
};
} // namespace autodrive_msgs

#endif
