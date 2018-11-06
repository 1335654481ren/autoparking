#ifndef STD_MSGS_UINT8MULTIARRAY_H
#define STD_MSGS_UINT8MULTIARRAY_H

#include "std_msgs_MultiArrayLayout.h"
#include <string>
#include <vector>
using namespace std;

namespace std_msgs {

class UInt8MultiArray
{
public:
  UInt8MultiArray() {};
  ~UInt8MultiArray() {};

  std_msgs::MultiArrayLayout layout;
  vector<unsigned char> data;
};
} // namespace std_msgs

#endif
