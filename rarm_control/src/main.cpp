#include "../include/main.h"

int main (int argc, char **argv){

  ros::init(argc,argv,"rarm_control_node");
  ros::NodeHandle node;

  ROS_INFO("Test0");

  TransMatrix *tr0 = new TransMatrix(1.0);
  double test_arr[4][4];
  tr0->getTMatrix(test_arr);

  for(int i=0; i<4; i++)
  {
    for(int j=0; j<4; j++)
      std::cout<<test_arr[i][j]<<" ";
    std::cout<<std::endl;
  }

  TransMatrix *tr1 = new TransMatrix( 0.0,  1.0,  2.0,   3.0,
                                      4.0,  5.0,  6.0,   7.0,
                                      8.0,  9.0, 10.0,  11.0,
                                     12.0, 13.0, 14.0,  15.0);

  tr1->getTMatrix(test_arr);

  std::cout<<""<<std::endl;
  for(int i=0; i<4; i++)
  {
    for(int j=0; j<4; j++)
      std::cout<<test_arr[i][j]<<" ";
    std::cout<<std::endl;
  }

  double r = 0.0;
  double p = 0.0;
  double y = 0.0;

  TransMatrix result;
  result = TransMatrix::DHToM(0.0, 0.0, L1, 0.0) * TransMatrix::DHToM(-M_PI/2, 0.0, 0.0, -M_PI/2);
  result.getRPY(r, p, y);

  std::cout<<"R: "<<r<<"\tP: "<<p<<"\tYaw: "<<y<<std::endl;

/*
  Kinematics *kn0 = new Kinematics();
  Kinematics *kn1 = new Kinematics( 0.0,  1.0,  2.0,   3.0,
                                    4.0,  5.0,  6.0,   7.0,
                                    8.0,  9.0, 10.0,  11.0,
                                   12.0, 13.0, 14.0,  15.0);

  double arr[] = { 0.0,  1.0,  2.0,   3.0,
                   4.0,  5.0,  6.0,   7.0,
                   8.0,  9.0, 10.0,  11.0,
                  12.0, 13.0, 14.0,  15.0};
  Kinematics *kn2 = new Kinematics(arr);

  double arr1[][4] = {{ 0.0,  1.0,  2.0,   3.0},
                      { 4.0,  5.0,  6.0,   7.0},
                      { 8.0,  9.0, 10.0,  11.0},
                      {12.0, 13.0, 14.0,  15.0}};

  Kinematics *kn3 = new Kinematics(arr1);
  *kn3 += *kn2;
  //*kn3 = (2.0  * ((Kinematics const) ref)) ;

  Kinematics kn4;
  //kn4 = Kinematics::DHToM(0.0, 0.0, L1, 0.0);
  kn4 = Kinematics::getFK(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  double test_arr[4][4];
  kn4.getTMatrix(test_arr);

  double r = 0.0;
  double p = 0.0;
  double y = 0.0;

  Kinematics result;
  result = Kinematics::DHToM(0.0, 0.0, L1, 0.0) * Kinematics::DHToM(-M_PI/2, 0.0, 0.0, -M_PI/2);
           //Kinematics::DHToM(0.0, L2, 0.0, j_angles[2]) * Kinematics::DHToM(0.0, L3, 0.0, j_angles[3]) *
  result.getRPY(r, p, y);

  std::cout<<""<<std::endl;
  for(int i=0; i<4; i++)
  {
    for(int j=0; j<4; j++)
      std::cout<<test_arr[i][j]<<" ";
    std::cout<<std::endl;
  }

  std::cout<<"Manipulator length: "<<L1+L2+L3+L4+L5+F0+F1<<std::endl;

  std::cout<<"R: "<<r<<"\tP: "<<p<<"\tYaw: "<<y<<std::endl;
*/
  return 0;

}
