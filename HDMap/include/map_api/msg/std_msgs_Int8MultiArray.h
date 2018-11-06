#ifndef STD_MSGS_INT8MULTIARRAY_H
#define STD_MSGS_INT8MULTIARRAY_H

#include "std_msgs_MultiArrayLayout.h"
#include <string>
#include <vector>
using namespace std;

namespace std_msgs {

class Int8MultiArray
{
public:
  Int8MultiArray() {};
  ~Int8MultiArray() {};

  std_msgs::MultiArrayLayout layout;
  vector<signed char> data;
};
} // namespace std_msgs

#endif
