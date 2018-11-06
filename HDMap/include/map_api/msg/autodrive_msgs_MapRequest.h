#ifndef AUTODRIVE_MSGS_MAPREQUEST_H
#define AUTODRIVE_MSGS_MAPREQUEST_H

#include "std_msgs_Header.h"
#include <string>
#include <vector>
using namespace std;

namespace autodrive_msgs {

class MapRequest
{
public:
  MapRequest() {};
  ~MapRequest() {};

  std_msgs::Header header;
  vector<string> roadids;
};
} // namespace autodrive_msgs

#endif
