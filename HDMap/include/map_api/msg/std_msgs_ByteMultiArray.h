#ifndef STD_MSGS_BYTEMULTIARRAY_H
#define STD_MSGS_BYTEMULTIARRAY_H

#include "std_msgs_MultiArrayLayout.h"
#include <string>
#include <vector>
using namespace std;

namespace std_msgs {

class ByteMultiArray
{
public:
  ByteMultiArray() {};
  ~ByteMultiArray() {};

  std_msgs::MultiArrayLayout layout;
  vector<signed char> data;
};
} // namespace std_msgs

#endif
