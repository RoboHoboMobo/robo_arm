#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
//#include "rarm_ctrl_msg/Ctrl_msg.h"
#include "rarm_ctrl_vector_msg/Ctrl_vector_msg.h"

#include <string>
#include <iostream>
#include <ncurses.h>

enum KEYS   // NCurses key codes
{
  // WASD
  WL = 119, // L means lower case
  W  = 87,  // Upper case
  AL = 97,
  A  = 65,
  SL = 115,
  S  = 83,
  DL = 100,
  D  = 68,

  // EQ RF OP key pairs
  EL = 101,
  E  = 69,
  QL = 113,
  Q  = 81,
  RL = 114,
  R  = 82,
  FL = 102,
  F  = 70,
  OL = 111,
  O  = 79,
  PL = 112,
  P  = 80,

  // Ctrl + C
  QUIT = 3
};

enum COMMANDS
{
  HALT,
  FORWARD,
  BACKWARD,
  RIGHT,
  LEFT,
  UP,
  DOWN,
  SPINUP,
  SPINDOWN,
  GRIP,
  UNGRIP
};

