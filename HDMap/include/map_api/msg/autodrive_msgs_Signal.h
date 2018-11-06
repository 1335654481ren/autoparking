#ifndef AUTODRIVE_MSGS_SIGNAL_H
#define AUTODRIVE_MSGS_SIGNAL_H

#include "std_msgs_Header.h"
#include <string>
#include <vector>
using namespace std;

namespace autodrive_msgs {

class Signal
{
public:
  Signal() {};
  ~Signal() {};

  enum {
    GREEN = 0,
    YELLOW = 1,
    RED = 2
  };

  std_msgs::Header header;
  string id;
  string type;
  string subtype;
  int color;
  double lefttime;
};
} // namespace autodrive_msgs

#endif
