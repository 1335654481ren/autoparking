#ifndef AUTODRIVE_MSGS_CARWORKINGSTATUS_H
#define AUTODRIVE_MSGS_CARWORKINGSTATUS_H

#include "std_msgs_Header.h"
#include <string>
#include <vector>
using namespace std;

namespace autodrive_msgs {

class CarWorkingStatus
{
public:
  CarWorkingStatus() {};
  ~CarWorkingStatus() {};

  std_msgs::Header header;
  int identifier;
  int status;
  int park_confirm;
  int get_confirm;
};
} // namespace autodrive_msgs

#endif
