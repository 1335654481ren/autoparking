#ifndef VISUALIZATION_MSGS_MENUENTRY_H
#define VISUALIZATION_MSGS_MENUENTRY_H

#include <string>
#include <vector>
using namespace std;

namespace visualization_msgs {

class MenuEntry
{
public:
  MenuEntry() {};
  ~MenuEntry() {};

  enum {
    FEEDBACK = 0,
    ROSRUN = 1,
    ROSLAUNCH = 2
  };

  unsigned int id;
  unsigned int parent_id;
  string title;
  string command;
  unsigned char command_type;
};
} // namespace visualization_msgs

#endif
