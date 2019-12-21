#include "trans_matrix.h"

#define L1 0.041  // Links lengts
#define L2 0.090
#define L3 0.090
#define L4 0.090
#define L5 0.049
#define F0 0.0459 // Gripper fingers' lengths
#define F1 0.060


class Kinematics
{
private:
  double th1, th2, th3, th4, th5, th6;
public:
  Kinematics(double th1 = 0.0, double th2 = 0.0, double th3 = 0.0,
             double th4 = 0.0, double th5 = 0.0, double th6 = 0.0);

  static TransMatrix getFK(double th1, double th2, double th3, double th4, double th5, double th6);
  static TransMatrix getFK(double j_angles[6]);
  static TransMatrix getFK(std::vector<double> j_angles);

};

