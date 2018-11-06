#ifndef STD_MSGS_UINT64MULTIARRAY_H
#define STD_MSGS_UINT64MULTIARRAY_H

#include "std_msgs_MultiArrayLayout.h"
#include <string>
#include <vector>
using namespace std;

namespace std_msgs {

class UInt64MultiArray
{
public:
  UInt64MultiArray() {};
  ~UInt64MultiArray() {};

  std_msgs::MultiArrayLayout layout;
  vector<unsigned long long> data;
};
} // namespace std_msgs

#endif
