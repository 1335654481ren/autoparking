#ifndef AUTODRIVE_MSGS_COMMAND_H
#define AUTODRIVE_MSGS_COMMAND_H

#include "std_msgs_Header.h"
#include <string>
#include <vector>
using namespace std;

namespace autodrive_msgs {

class Command
{
public:
  Command() {};
  ~Command() {};

  std_msgs::Header header;
  unsigned char command;
};
} // namespace autodrive_msgs

#endif
