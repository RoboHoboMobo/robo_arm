#include "../include/rarm_keyboard_control.h"

int main (int argc, char **argv){

  ros::init(argc,argv,"rarm_keyboard_control");
  ros::NodeHandle n;

  ros::Publisher ctrl_pub = n.advertise<rarm_ctrl_vector_msg::Ctrl_vector_msg>
                              ("rarm_keyboard_control", 1);

  // Curses library code
  initscr(); // Init new screen
  clear();   // Clear screen;
  noecho();  // Don't display input characters
  raw();     // Raw input without confirmation thru Enter key

  bool in_loop = true;
  ros::Rate rate(200); //Hz
  while(ros::ok() && in_loop)
  {
    clear();
    printw("***RARM_KEYBOARD_CONTROL_NODE***\n\n");
    printw("Rollover is not enable!\n");
    printw("Use 'WASD' to navigate in XY-plane\n");
    printw("Press 'E' to go up\n");
    printw("Press 'Q' to go down\n");
    printw("Press 'R' to rotate up\n");
    printw("Press 'F' to rotate down\n");
    printw("Press 'O' to grip\n");
    printw("Press 'P' to unclasp\n");
    printw("\nPress Ctrl+C to quit\n");

    rarm_ctrl_vector_msg::Ctrl_vector_msg ctrl_vec;
    ctrl_vec.ctrl_vector.clear();

    rarm_ctrl_msg::Ctrl_msg               ctrl_msg;
    ctrl_msg.dof_name.data = "";
    ctrl_msg.control_value.data = -1;

    int input_key = wgetch(stdscr);

    switch(input_key)
    {

    case W:
    case WL:
      ctrl_msg.dof_name.data = "x";
      ctrl_msg.control_value.data = 1;
      break;

    case S:
    case SL:
      ctrl_msg.dof_name.data = "x";
      ctrl_msg.control_value.data = 0;
      break;

    case A:
    case AL:
      ctrl_msg.dof_name.data = "y";
      ctrl_msg.control_value.data = 1;
      break;

    case D:
    case DL:
      ctrl_msg.dof_name.data = "y";
      ctrl_msg.control_value.data = 0;
      break;

    case E:
    case EL:
      ctrl_msg.dof_name.data = "z";
      ctrl_msg.control_value.data = 1;
      break;

    case Q:
    case QL:
      ctrl_msg.dof_name.data = "z";
      ctrl_msg.control_value.data = 0;
      break;

    case R:
    case RL:
      ctrl_msg.dof_name.data = "spin";
      ctrl_msg.control_value.data = 1;
      break;

    case F:
    case FL:
      ctrl_msg.dof_name.data = "spin";
      ctrl_msg.control_value.data = 0;
      break;

    case O:
    case OL:
      ctrl_msg.dof_name.data = "grip";
      ctrl_msg.control_value.data = 1;
      break;

    case P:
    case PL:
      ctrl_msg.dof_name.data = "grip";
      ctrl_msg.control_value.data = 0;
      break;

    case QUIT:
      in_loop = false;
      break;
    }

    ctrl_vec.ctrl_vector.push_back(ctrl_msg);
    ctrl_pub.publish(ctrl_vec);


    ros::spinOnce();
    rate.sleep();
  }

  noraw();
  cbreak();
  refresh();

  endwin(); // Quit curses mode

  //system("clear");

  return 0;
}
