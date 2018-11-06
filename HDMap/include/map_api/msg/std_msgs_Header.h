#ifndef STD_MSGS_HEADER_H
#define STD_MSGS_HEADER_H

#include <string>
#include <vector>
using namespace std;

namespace std_msgs {

class Header
{
public:
  Header() {};
  ~Header() {};

  unsigned int seq;
  double stamp;
  string frame_id;
};
} // namespace std_msgs

#endif
