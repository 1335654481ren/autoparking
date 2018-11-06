#ifndef AUTODRIVE_MSGS_DECISION_H
#define AUTODRIVE_MSGS_DECISION_H

#include "std_msgs_Header.h"
#include <string>
#include <vector>
using namespace std;

namespace autodrive_msgs {

class Decision
{
public:
  Decision() {};
  ~Decision() {};

  std_msgs::Header header;
  double left;
  double keep;
  double right;
};
} // namespace autodrive_msgs

#endif
