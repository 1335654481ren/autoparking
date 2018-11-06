#ifndef AUTODRIVE_MSGS_SIGNALS_H
#define AUTODRIVE_MSGS_SIGNALS_H

#include "std_msgs_Header.h"
#include "autodrive_msgs_Signal.h"
#include <string>
#include <vector>
using namespace std;

namespace autodrive_msgs {

class Signals
{
public:
  Signals() {};
  ~Signals() {};

  std_msgs::Header header;
  vector<autodrive_msgs::Signal> signals;
};
} // namespace autodrive_msgs

#endif
