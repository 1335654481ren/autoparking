#ifndef STD_MSGS_INT32MULTIARRAY_H
#define STD_MSGS_INT32MULTIARRAY_H

#include "std_msgs_MultiArrayLayout.h"
#include <string>
#include <vector>
using namespace std;

namespace std_msgs {

class Int32MultiArray
{
public:
  Int32MultiArray() {};
  ~Int32MultiArray() {};

  std_msgs::MultiArrayLayout layout;
  vector<int> data;
};
} // namespace std_msgs

#endif
