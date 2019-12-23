#include "../include/main.h"

int main (int argc, char **argv){

  ros::init(argc,argv,"rarm_control_node");
  ros::NodeHandle node;

  ROS_INFO("Test0");
/*
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

  std::ccout<<""<<std::endl;
*/
  double test_arr[4][4];
  TransMatrix tr0;
  tr0 = Kinematics::solveFK(0.0, 0.0, 0.0, 0.0, -M_PI/2, 0.0);

  tr0.getTMatrix(test_arr);

  for(int i=0; i<4; i++)
  {
    for(int j=0; j<4; j++)
      std::cout<<test_arr[i][j]<<" ";
    std::cout<<std::endl;
  }
  std::cout<<""<<std::endl;

  double r = 0.0;
  double p = 0.0;
  double y = 0.0;

  tr0.getRPY(r, p, y);
  std::cout<<"R: "<<r<<"\tP: "<<p<<"\tYaw: "<<y<<std::endl;
  std::cout<<""<<std::endl;

  TransMatrix tr1(0.0, 1.0, 0.0, 0.0,
                  0.0, 0.0, 1.0, 0.0,
                  1.0, 0.0, 0.0, 0.4659,
                  0.0, 0.0, 0.0, 1.0);

  Kinematics kn0(tr1);
  kn0.setK(0.9);

  std::vector<double> vec;
  vec = kn0.getJAngles();

  for(int i=0; i<5; i++)
  {
    std::cout<<vec[i]*180.0/M_PI<<" ";
  }
  std::cout<<""<<std::endl;

  TransMatrix m = kn0.getMatrix();

  std::cout<<"\ngetMatrix"<<std::endl;
  for(int i=0; i<4; i++)
  {
    for(int j=0; j<4; j++)
      std::cout<<m.getValue(i,j)<<" ";
    std::cout<<std::endl;
  }


  return 0;

}
