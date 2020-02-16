#include "../include/rarm_keyboard_control.h"

int main (int argc, char **argv){

  ros::init(argc,argv,"rarm_keyboard_control");
  ros::NodeHandle node;

  ros::Publisher get_key_pub = node.advertise<std_msgs::Int32>("rarm_keyboard_control", 1000);

  ROS_INFO("Use keyboard to control the manipulator");
  ROS_INFO("Use 'WASD' to navigate in XY-plane");
  ROS_INFO("Press 'E' to go up");
  ROS_INFO("Press 'Q' to go down");
  ROS_INFO("Press 'E' to go up");
  ROS_INFO("Press 'R' to rotate up");
  ROS_INFO("Press 'F' to rotate down");
  ROS_INFO("Press 'O' to grip");
  ROS_INFO("Press 'P' to unclasp");


  ros::Rate rate(30); //Hz

  while(ros::ok()){

    std_msgs::Int32 control_msg;
    //std::stringstream sstream;

    // Curses library code
    WINDOW *w;
    w=initscr();     // init new screen
    cbreak();        // use cbreack call to make terminal send all keystrokes directly
    nodelay(w,true); // non-blocking call for getch

    refresh();       // push from buffer to the real terminal
    endwin();

    char input_key=getch();

    switch(input_key)
    {
    case 'w':
    case 'W':
      control_msg.data = FORWARD;
      break;

    case 's':
    case 'S':
      control_msg.data = BACKWARD;
      break;

    case 'a':
    case 'A':
      control_msg.data = LEFT;
      break;

    case 'd':
    case 'D':
      control_msg.data = RIGHT;
      break;

    case 'q':
    case 'Q':
      control_msg.data = DOWN;
      break;

    case 'e':
    case 'E':
      control_msg.data = UP;
      break;

    case 'r':
    case 'R':
      control_msg.data = SPINUP;
      break;

    case 'f':
    case 'F':
      control_msg.data = SPINDOWN;
      break;

    case 'o':
    case 'O':
      control_msg.data = GRIP;
      break;

    case 'p':
    case 'P':
      control_msg.data = UNGRIP;
      break;

    default:
      control_msg.data = HALT;
      break;

    }

    get_key_pub.publish(control_msg);

    ros::spinOnce();
    rate.sleep();
  }

  //system("clear");

  return 0;
}
