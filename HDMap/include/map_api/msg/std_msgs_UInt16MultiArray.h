#ifndef STD_MSGS_UINT16MULTIARRAY_H
#define STD_MSGS_UINT16MULTIARRAY_H

#include "std_msgs_MultiArrayLayout.h"
#include <string>
#include <vector>
using namespace std;

namespace std_msgs {

class UInt16MultiArray
{
public:
  UInt16MultiArray() {};
  ~UInt16MultiArray() {};

  std_msgs::MultiArrayLayout layout;
  vector<unsigned short> data;
};
} // namespace std_msgs

#endif
