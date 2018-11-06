#ifndef AUTODRIVE_MSGS_MPC_H
#define AUTODRIVE_MSGS_MPC_H

#include <string>
#include <vector>
using namespace std;

namespace autodrive_msgs {

class Mpc
{
public:
  Mpc() {};
  ~Mpc() {};

  float x;
  float y;
  float theta;
  float v;
  float dtheta;
  float epsi;
  float acc;
  float t1;
  float t2;
  float t3;
  float t4;
  float t5;
};
} // namespace autodrive_msgs

#endif
