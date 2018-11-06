#ifndef STD_MSGS_INT64MULTIARRAY_H
#define STD_MSGS_INT64MULTIARRAY_H

#include "std_msgs_MultiArrayLayout.h"
#include <string>
#include <vector>
using namespace std;

namespace std_msgs {

class Int64MultiArray
{
public:
  Int64MultiArray() {};
  ~Int64MultiArray() {};

  std_msgs::MultiArrayLayout layout;
  vector<long long> data;
};
} // namespace std_msgs

#endif
