#ifndef STD_MSGS_UINT32MULTIARRAY_H
#define STD_MSGS_UINT32MULTIARRAY_H

#include "std_msgs_MultiArrayLayout.h"
#include <string>
#include <vector>
using namespace std;

namespace std_msgs {

class UInt32MultiArray
{
public:
  UInt32MultiArray() {};
  ~UInt32MultiArray() {};

  std_msgs::MultiArrayLayout layout;
  vector<unsigned int> data;
};
} // namespace std_msgs

#endif
