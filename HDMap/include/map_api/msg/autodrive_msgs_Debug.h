#ifndef AUTODRIVE_MSGS_DEBUG_H
#define AUTODRIVE_MSGS_DEBUG_H

#include <string>
#include <vector>
using namespace std;

namespace autodrive_msgs {

class Debug
{
public:
  Debug() {};
  ~Debug() {};

  float engine_torque;
  float flvelocity;
  float frvelocity;
};
} // namespace autodrive_msgs

#endif
