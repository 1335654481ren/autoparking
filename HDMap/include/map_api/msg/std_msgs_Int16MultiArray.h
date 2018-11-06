#ifndef STD_MSGS_INT16MULTIARRAY_H
#define STD_MSGS_INT16MULTIARRAY_H

#include "std_msgs_MultiArrayLayout.h"
#include <string>
#include <vector>
using namespace std;

namespace std_msgs {

class Int16MultiArray
{
public:
  Int16MultiArray() {};
  ~Int16MultiArray() {};

  std_msgs::MultiArrayLayout layout;
  vector<short> data;
};
} // namespace std_msgs

#endif
