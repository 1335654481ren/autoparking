#ifndef AUTODRIVE_MSGS_OPENDRIVE_OBJECT_MSG_H
#define AUTODRIVE_MSGS_OPENDRIVE_OBJECT_MSG_H

#include "autodrive_msgs_opendrive_object_shape_msg.h"
#include <string>
#include <vector>
using namespace std;

namespace autodrive_msgs {

class opendrive_object_msg
{
public:
  opendrive_object_msg() {};
  ~opendrive_object_msg() {};

  string id;
  string type;
  unsigned char hasSubtype;
  string subtype;
  vector<autodrive_msgs::opendrive_object_shape_msg> shape;
};
} // namespace autodrive_msgs

#endif
